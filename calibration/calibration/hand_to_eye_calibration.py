#!/usr/bin/env python3

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import MoveItPy

PLANNING_GROUP = "ur5e_arm"
TIP_LINK = "ur5e_tool0"

# Pose des ArUco-Kalibriertargets relativ zum "table"-Frame (liegt flach auf
# dem Tisch, 1mm über der Oberfläche gegen Z-Fighting mit dem Tischmesh).
ARUCO_BOARD_FRAME = "table"
ARUCO_BOARD_POSITION = (0.0, 0.6, 0.021)
ARUCO_BOARD_MESH_RESOURCE = "package://calibration/meshes/aruco_board.obj"

# Board-Abmessungen aus aruco_marker.py: 4x0.04m Marker + 3x0.01m Abstand
# = 0.19m breit (x), 6x0.04m + 5x0.01m = 0.29m hoch (y).
BOARD_HALF_WIDTH = 0.095
BOARD_HALF_HEIGHT = 0.145
EDGE_HEIGHT_ABOVE_BOARD = 0.20  # m, wie weit der Greifer über dem Rand schwebt

# Roll um die lokale Anfahrachse (Z), ändert nicht die Blickrichtung, nur wie
# der Greifer um diese Achse gedreht ist (z.B. wohin der Stromanschluss zeigt).
GRIPPER_ROLL_OFFSET_DEG = -180

# 3 Punkte um das Board: vorne (Roboter-Seite), rechts, links -- jeweils
# (dx, dy) relativ zu board_center, in der Tisch-Ebene.
BOARD_SIDE_POINTS = [
    ("vorne", (0.0, -BOARD_HALF_HEIGHT)),
    ("rechts", (BOARD_HALF_WIDTH, 0.0)),
    ("links", (-BOARD_HALF_WIDTH, 0.0)),
]

# Real geteachte Posen: Freedrive -> `ros2 topic echo /joint_states` -> eintragen.
# Für Hand-Eye: 10-15 Posen, Kalibriertarget aus verschiedenen Winkeln/Distanzen,
# viel Rotationsvielfalt um alle Achsen.
CALIB_POSES = [
    {
        "ur5e_shoulder_pan_joint":  1.396,
        "ur5e_shoulder_lift_joint": -1.92,
        "ur5e_elbow_joint":          1.92,
        "ur5e_wrist_1_joint":       -1.57,
        "ur5e_wrist_2_joint":       -1.57,
        "ur5e_wrist_3_joint":        0.0,
    },
    # ... weitere geteachte Posen hier
]

SETTLE_TIME = 1.5  # s Nachschwingen vor Sample-Aufnahme


def _look_at_quaternion(cam_pos, target_pos):
    """Quaternion, sodass die lokale +Z-Achse (Anfahrrichtung von tool0) von
    cam_pos auf target_pos zeigt. Rechtshändige, in sich konsistente
    Basiskonstruktion (bereits gegen den Geradenach-unten-Fall verifiziert:
    liefert für cam_pos direkt über target_pos exakt die 180°-um-X-Quaternion)."""
    forward = target_pos - cam_pos
    forward = forward / np.linalg.norm(forward)

    up_ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(forward, up_ref)) > 0.99:
        up_ref = np.array([0.0, 1.0, 0.0])

    right = np.cross(up_ref, forward)
    right /= np.linalg.norm(right)
    true_up = np.cross(forward, right)

    rot = np.column_stack((right, true_up, forward))  # lokale X,Y,Z -> Welt/Frame

    tr = np.trace(rot)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    else:
        i = int(np.argmax([rot[0, 0], rot[1, 1], rot[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
    return qx, qy, qz, qw


def _quat_mul(q1, q2):
    """Hamilton-Produkt q1*q2, beide als (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _roll_z_quaternion(angle_deg):
    """Rotation um die LOKALE Z-Achse (Anfahrrichtung) -- ändert nur den Roll,
    nicht die Blickrichtung."""
    half = math.radians(angle_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def board_side_pose_targets():
    """PoseStamped-Ziele für 3 Punkte um das Board (vorne/rechts/links, s.
    BOARD_SIDE_POINTS), Werkzeug jeweils auf board_center ausgerichtet.

    Direkt im ARUCO_BOARD_FRAME (also relativ zur Tischplatte, NICHT relativ
    zu base_link) gestamped -- MoveIt übernimmt beim Planen selbst das TF-
    Lookup in den Modell-Frame. EDGE_HEIGHT_ABOVE_BOARD ist also direkt die
    Höhe über der Tischplatte, unabhängig davon, wo base_link/world liegt.

    Board-Mittelpunkt und Kamerapositionen liegen beide im selben Frame, die
    Look-At-Berechnung braucht also keine Frame-Transformation.
    """
    board_center = np.array(ARUCO_BOARD_POSITION)
    targets = []
    for label, (dx, dy) in BOARD_SIDE_POINTS:
        cam_pos = board_center + np.array([dx, dy, EDGE_HEIGHT_ABOVE_BOARD])
        look_at_quat = _look_at_quaternion(cam_pos, board_center)
        qx, qy, qz, qw = _quat_mul(look_at_quat, _roll_z_quaternion(GRIPPER_ROLL_OFFSET_DEG))

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = ARUCO_BOARD_FRAME
        pose_stamped.pose.position.x = cam_pos[0]
        pose_stamped.pose.position.y = cam_pos[1]
        pose_stamped.pose.position.z = cam_pos[2]
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        targets.append((label, pose_stamped))
    return targets


def publish_aruco_board_marker(node):
    """Zeigt das ArUco-Kalibriertarget als texturiertes Mesh in RViz an.

    Nur Visualisierung (kein Collision-Objekt in der Planning Scene).
    Wird per Timer dauerhaft neu gepublisht (statt einmalig), damit es
    unabhängig davon ankommt, wann RViz' Subscriber discovered/gematcht hat.
    """
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(Marker, "/aruco_board_marker", qos)

    marker = Marker()
    marker.header.frame_id = ARUCO_BOARD_FRAME
    marker.ns = "aruco_board"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = ARUCO_BOARD_MESH_RESOURCE
    marker.mesh_use_embedded_materials = True
    marker.action = Marker.ADD
    marker.pose.position.x = ARUCO_BOARD_POSITION[0]
    marker.pose.position.y = ARUCO_BOARD_POSITION[1]
    marker.pose.position.z = ARUCO_BOARD_POSITION[2]
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0

    def republish():
        marker.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(marker)

    republish()
    node.create_timer(1.0, republish)


def plan_and_execute(robot, planning_component, logger, sleep_time=0.0):
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    if plan_result:
        logger.info("Executing plan")
        robot.execute(plan_result.trajectory, controllers=[])
        time.sleep(sleep_time)
        return True

    logger.error("Planning failed")
    return False


def main():
    rclpy.init()
    logger = get_logger("hand_to_eye_calibration")

    marker_node = Node("aruco_board_marker_publisher")
    publish_aruco_board_marker(marker_node)
    marker_spin_thread = threading.Thread(
        target=rclpy.spin, args=(marker_node,), daemon=True
    )
    marker_spin_thread.start()

    ur5e = MoveItPy(node_name="moveit_py")
    ur5e_arm = ur5e.get_planning_component(PLANNING_GROUP)
    robot_model = ur5e.get_robot_model()
    jmg = robot_model.get_joint_model_group(PLANNING_GROUP)
    logger.info("MoveItPy instance created")

    ur5e_arm.set_workspace(
        min_x=0.0, min_y=-0.5, min_z=0.0,
        max_x=0.8, max_y=0.5, max_z=1.0,
    )

    # Optional: definierter Start über deinen SRDF-State
    ur5e_arm.set_start_state_to_current_state()
    ur5e_arm.set_goal_state(configuration_name="ready")
    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=1.0)

    side_targets = board_side_pose_targets()
    total_poses = len(side_targets) + len(CALIB_POSES)
    collected = 0

    for i, (label, pose_stamped) in enumerate(side_targets):
        logger.info(f"Kalibrierpose {i + 1}/{total_poses} ('{label}')")

        ur5e_arm.set_start_state_to_current_state()
        ur5e_arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=TIP_LINK)

        if not plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=SETTLE_TIME):
            logger.warn(f"Pose {i + 1} ('{label}') übersprungen")
            continue

        # === HIER: Sample aufnehmen ===
        logger.info(f"Pose {i + 1}: Sample aufgenommen")
        collected += 1

    for i, joint_values in enumerate(CALIB_POSES):
        pose_num = len(side_targets) + i + 1
        logger.info(f"Kalibrierpose {pose_num}/{total_poses}")

        ur5e_arm.set_start_state_to_current_state()

        goal_state = RobotState(robot_model)
        goal_state.set_to_default_values()
        goal_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=goal_state, joint_model_group=jmg
        )
        ur5e_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        if not plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=SETTLE_TIME):
            logger.warn(f"Pose {pose_num} übersprungen")
            continue

        # === HIER: Sample aufnehmen ===
        # 1. Bild von der Handgelenkskamera triggern
        # 2. TF base_link -> tool0 loggen (Zeitstempel-konsistent zum Bild!)
        logger.info(f"Pose {pose_num}: Sample aufgenommen")
        collected += 1

    logger.info(f"Fertig: {collected}/{total_poses} Samples")

    # Bekannter, ungefixter moveit_py-Bug: Der Prozess segfault't beim
    # Zerstören von MoveItCpp (DDS-Datareader-Teardown), siehe
    # https://github.com/moveit/moveit2/issues/2693. Passiert erst NACHDEM
    # alle Posen erfolgreich abgefahren und Samples aufgenommen wurden ->
    # unschön, aber harmlos für dieses Skript.
    rclpy.shutdown()
    marker_spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()