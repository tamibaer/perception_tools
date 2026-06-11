# Perception Tools 📸
Software prerequisites:
```bash
sudo apt install ros-jazzy-librealsense2*
```

# Camera Calibration
Camera calibration is essential for accurately determining the camera extrinsics, which are crucial for precise robotic grasping tasks.
## Eye-to-Hand Calibration
Launch of eye-to-hand calibration + D415 Camera
```bash
ros2 launch calibration eye_to_hand_calibration.launch.py
```
## Eye-in-Hand Calibration

## Intel Realsense
General Launch (one camera automatic detection)

```bash
ros2 launch realsense2_camera rs_launch.py 
```

Wrist Camera Launch

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=D405 serial_no:='"128422272705"'
```

Static Table Camera Launch

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=D415 serial_no:='"241222063543"'
```

