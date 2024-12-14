# ArUco Workspace ROI and Object Localization

This ROS2 package uses ArUco markers to define a region of interest (ROI) in a workspace and compute the real-world coordinates of objects within that region. It detects markers, estimates their poses, and calculates the object's actual position based on camera calibration data.

## Dependencies

- ROS 2 (Foxy, Galactic, Humble or later)
- OpenCV
- `cv_bridge`
- `sensor_msgs`

## Installation

### 1. Clone the repository

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Leeseunghun03/ros2-aruco-object-estimator.git aruco_workspace
```

### 2. Build

```bash
cd ~/ros2_ws && colcon build --symlink-install
```

### 3. Camera (RGBD)

```bash
ros2 launch realsense2_camera rs_align_depth_launch.py
```

### 4. Start Estimate
```bash
ros2 launch object_estimator object_estimator.py
```

## Example Use Case: Autonomous Feeding Robot
This package can be integrated into an autonomous feeding robot, where ArUco markers are used to estimate tray serving locations. The robot detects the markers placed on a tray and accurately calculates the serving positions, helping the robot to autonomously serve food to individuals in a predefined area. This capability can be used in healthcare, hospitality, and other industries that require automated feeding solutions.

## Video

![ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/e23888e0-d981-4aa4-a9bc-ae295598f9e6)
