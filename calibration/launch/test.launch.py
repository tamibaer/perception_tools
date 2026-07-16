# launch/hand_to_eye_calibration.launch.py in deinem calibration-Package
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot_table", package_name="robot_table_moveit_config")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines()
        .trajectory_execution()
        .moveit_cpp(
            file_path=get_package_share_directory(""
            "calibration")
            + "/config/test.yaml"
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("calibration"),
            "config",
            "config.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        rviz_node,
        Node(
            package="calibration",
            executable="test",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
    ])

