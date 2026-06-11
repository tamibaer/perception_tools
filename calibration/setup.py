from setuptools import find_packages, setup
from glob import glob

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dattelpalmara',
    maintainer_email='tamara.bergerhoff@mainfranken-racing.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'eye_to_hand_calibration = calibration.eye_to_hand_calibration:main',
            'hand_eye_calibration = calibration.hand_eye_calibration:main',
            'tool_pose_reader = calibration.tool_pose_reader:main',
            'aruco_circle_motion = calibration.aruco_circle_motion:main',
        ],
    },
)
