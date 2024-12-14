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
git clone <repository_url> aruco_workspace
