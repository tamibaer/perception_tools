#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Empty

# ---------------------------------------------------------------------------
# Konfiguration
# ---------------------------------------------------------------------------
BOARD_X, BOARD_Y, BOARD_Z = 0.006, 0.45, 0.02   # Board-Mittelpunkt [m]

ORBIT_RADIUS    = 0.28    # Kreisradius um das Board [m]
ORBIT_HEIGHT    = 0.4     # Z-Höhe des Tools (Vorderseite) [m]
Z_LIFT_FACTOR   = 0.55    # Wie stark back-Positionen angehoben werden [m/m]
N_WAYPOINTS     = 8       # Gleichmäßig auf 360° verteilt (je 45°)
START_ANGLE_DEG = 0.0     # 0° = rechts vom Board (+X Richtung)

VELOCITY_SCALE  = 0.1
MAX_STEP        = 0.01    # Interpolationsschrittgröße [m]
JUMP_THRESHOLD  = 5.0     # Verhindert Gelenk-Konfigurationswechsel
MIN_FRACTION    = 0.95
CAPTURE_DELAY   = 1.5     # Wartezeit für Datenerfassung [s]
# ---------------------------------------------------------------------------


def _look_at(pos: np.ndarray, target: np.ndarray) -> tuple[float, float, float]:
    """
    Kamera zeigt immer zum Board (tool0 = Kamera-Frame, Offset=0):
      roll  = π  → natürliche Greifhaltung, Kabel läuft am Arm entlang
      pitch → Neigungswinkel zum Board
      yaw   → azimutale Richtung zum Board
    """
    d = target - pos
    dx, dy, dz = d / np.linalg.norm(d)
    return (
        float(np.pi),
        float(np.arctan2(np.hypot(dx, dy), -dz)),
        float(np.arctan2(-dy, -dx)),
    )


def _make_pose(x, y, z, roll, pitch, yaw) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    p.orientation.x, p.orientation.y = float(q[0]), float(q[1])
    p.orientation.z, p.orientation.w = float(q[2]), float(q[3])
    return p


class ArucoCircleMotion(Node):

    def __init__(self):
        super().__init__('aruco_circle_motion')

        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.get_logger().info('Warte auf /compute_cartesian_path …')
        self.cart_cli.wait_for_service()

        self.exec_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.get_logger().info('Warte auf /execute_trajectory …')
        self.exec_cli.wait_for_server()

        self.capture_pub = self.create_publisher(Empty, '/calibration/capture', 10)

        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self._queue: list          = []
        self._pending: Pose | None = None
        self._retried              = False
        self._timer                = None
        self.pose_count            = 0

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def _start_state(self) -> RobotState:
        rs = RobotState()
        rs.joint_state.header.stamp = self.get_clock().now().to_msg()
        rs.joint_state.name         = list(self.joints.keys())
        rs.joint_state.position     = list(self.joints.values())
        return rs

    # ------------------------------------------------------------------
    # Queue aufbauen
    # ------------------------------------------------------------------

    def _build_queue(self):
        board = np.array([BOARD_X, BOARD_Y, BOARD_Z])
        start = np.deg2rad(START_ANGLE_DEG)

        # Vollkreis: N Punkte bei Schritt 360°/N.
        # Damit ist der Schritt von WP(N-1) zurück zu WP0 gleich groß
        # wie alle anderen Schritte → kein Singularitätsproblem beim Schließen.
        #
        # Kegelorbit: Positionen auf der "Rückseite" (nahe Roboter, y < BOARD_Y)
        # werden angehoben. Dadurch zeigt die Kamera mehr nach unten statt
        # geradeaus — vermeidet die Fast-Singularität, bei der Arm- und
        # Kamerarichtung parallel laufen.
        orbit_poses: list[Pose] = []
        for i in range(N_WAYPOINTS):
            th = start + 2 * np.pi * i / N_WAYPOINTS
            px = BOARD_X + ORBIT_RADIUS * np.cos(th)
            py = BOARD_Y + ORBIT_RADIUS * np.sin(th)
            z_lift = max(0.0, (BOARD_Y - py) * Z_LIFT_FACTOR)
            pos = np.array([px, py, ORBIT_HEIGHT + z_lift])
            orbit_poses.append(_make_pose(*pos, *_look_at(pos, board)))

        # Einfahrpose: senkrecht über dem Startpunkt
        wp0   = orbit_poses[0]
        entry = _make_pose(
            wp0.position.x, wp0.position.y, ORBIT_HEIGHT + 0.15,
            np.pi, 0.0, 0.0,
        )

        # 1. Einfahren
        self._queue.append(('move', entry))

        # 2. Vollkreis mit Captures
        for wp in orbit_poses:
            self._queue.append(('move', wp))
            self._queue.append(('capture', None))

        # 3. Kreis schließen: WP7 → WP0 (nur 45° Schritt, wie alle anderen)
        self._queue.append(('move', orbit_poses[0]))

        # 4. Zurück zur Einfahrpose → direkt neu startbar
        self._queue.append(('move', entry))

    # ------------------------------------------------------------------
    # Ausführungsschleife
    # ------------------------------------------------------------------

    def start(self):
        self._build_queue()
        self.get_logger().info(
            f'Start: {N_WAYPOINTS} Posen auf 360° Kreis | '
            f'r={ORBIT_RADIUS} m | v={VELOCITY_SCALE * 100:.0f}%'
        )
        self._next()

    def _next(self):
        if not self._queue:
            self.get_logger().info(
                f'Fertig — {self.pose_count}/{N_WAYPOINTS} Posen aufgenommen.'
            )
            return

        kind, data = self._queue.pop(0)

        if kind == 'move':
            self._pending = data
            self._retried = False
            p = data.position
            self.get_logger().info(f'→ ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
            self._plan(data)

        elif kind == 'capture':
            self.pose_count += 1
            self.get_logger().info(f'[{self.pose_count}/{N_WAYPOINTS}] Capture')
            self.capture_pub.publish(Empty())
            self._timer = self.create_timer(CAPTURE_DELAY, self._timer_cb)

    def _timer_cb(self):
        self._timer.cancel()
        self._timer = None
        self._next()

    # ------------------------------------------------------------------
    # Kartesische Planung + Ausführung
    # ------------------------------------------------------------------

    def _plan(self, target: Pose, jump: float = JUMP_THRESHOLD):
        req = GetCartesianPath.Request()
        req.header.frame_id   = 'ur5e_base_link'
        req.header.stamp      = self.get_clock().now().to_msg()
        req.group_name        = 'ur5e_arm'
        req.link_name         = 'ur5e_tool0'
        req.waypoints         = [target]
        req.max_step          = MAX_STEP
        req.jump_threshold    = jump
        req.avoid_collisions  = True
        req.max_velocity_scaling_factor     = VELOCITY_SCALE
        req.max_acceleration_scaling_factor = VELOCITY_SCALE
        req.start_state       = self._start_state()
        self.cart_cli.call_async(req).add_done_callback(self._plan_cb)

    def _plan_cb(self, fut):
        resp = fut.result()

        if resp is None:
            self.get_logger().error('Kein Service-Response — überspringe')
            self._pending = None
            self._next()
            return

        frac = resp.fraction
        self.get_logger().info(f'Pfad: {frac * 100:.1f}%')

        if frac < MIN_FRACTION:
            if not self._retried and self._pending is not None:
                self._retried = True
                self.get_logger().warn(f'Fraction={frac:.2f} — Retry ohne Jump-Check')
                self._plan(self._pending, jump=0.0)
                return
            self.get_logger().error(f'Nur {frac * 100:.1f}% erreichbar — überspringe')
            self._pending = None
            self._next()
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution
        self.exec_cli.send_goal_async(goal).add_done_callback(self._exec_resp_cb)

    def _exec_resp_cb(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('ExecuteTrajectory abgelehnt — überspringe')
            self._pending = None
            self._next()
            return
        gh.get_result_async().add_done_callback(self._exec_result_cb)

    def _exec_result_cb(self, fut):
        code = fut.result().result.error_code.val
        if code != 1:
            self.get_logger().warn(f'Ausführung fehlgeschlagen (code={code})')
        self._pending = None
        self._next()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCircleMotion()
    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
