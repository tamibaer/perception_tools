# Perception Tools 📸
Software prerequisites:
```bash
sudo apt install ros-jazzy-librealsense2*
```

# Camera Calibration

## Eye-to-Hand Calibration

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

