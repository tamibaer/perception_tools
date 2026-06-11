from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'D415',
            'serial_no': '"241222063543"'
        }.items()
    )

    calibration_node = Node(
        package='calibration',
        executable='eye_to_hand_calibration',
        name='eye_to_hand_calibration',
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        calibration_node
    ])