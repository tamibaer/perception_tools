#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import MoveItPy

# x,y,z pose aruco board in base_link frame (table)
ARUCO_BOARD_FRAME = "table"
ARUCO_BOARD_POSITION = (0.0, 0.55, 0.021)
ARUCO_BOARD_MESH_RESOURCE = "package://calibration/meshes/aruco_board.obj"

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

SETTLE_TIME = 1.5  #in [s] Nachschwingen vor Sample-Aufnahme


def publish_aruco_board_marker(node):
    """Zeigt das ArUco-Kalibriertarget als texturiertes Mesh in RViz an.

    Wird per Timer dauerhaft neu gepublisht (statt einmalig), damit es
    unabhängig davon ankommt.
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
    ur5e_arm = ur5e.get_planning_component("ur5e_arm")
    robot_model = ur5e.get_robot_model()
    jmg = robot_model.get_joint_model_group("ur5e_arm")
    logger.info("MoveItPy instance created")

    ur5e_arm.set_workspace(
        min_x=0.0, min_y=-0.5, min_z=0.0,
        max_x=0.8, max_y=0.5, max_z=1.0,
    )

    ur5e_arm.set_start_state_to_current_state()
    ur5e_arm.set_goal_state(configuration_name="ready")
    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=1.0)

    collected = 0
    for i, joint_values in enumerate(CALIB_POSES):
        logger.info(f"Kalibrierpose {i + 1}/{len(CALIB_POSES)}")

        ur5e_arm.set_start_state_to_current_state()

        goal_state = RobotState(robot_model)
        goal_state.set_to_default_values()
        goal_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=goal_state, joint_model_group=jmg
        )
        ur5e_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        if not plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=SETTLE_TIME):
            logger.warn(f"Pose {i + 1} übersprungen")
            continue

        # === HIER: Sample aufnehmen ===
        # 1. Bild von der Handgelenkskamera triggern
        # 2. TF base_link -> tool0 loggen (Zeitstempel-konsistent zum Bild!)
        logger.info(f"Pose {i + 1}: Sample aufgenommen")
        collected += 1

    logger.info(f"Fertig: {collected}/{len(CALIB_POSES)} Samples")

    # Cleanup (Error am Ende vom Script ist normal)
    rclpy.shutdown()
    marker_spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()