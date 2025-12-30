# Aruco detection and angular visual servoing in Gazebo & ROS 2

**Assignment 1 — Experimental Robotics Lab**  
Authors: Gian Marco Balia, Christian Negri Ravera, Francesca Amato, Arian Tavousi, Milad Rabiei

**Short description:**  
Spawn a robot in a Gazebo world with 5 ArUco markers placed in a circle. The system detects all markers, then, in ascending order of marker ID, rotates the robot to center each marker in the image (visual servoing phase), publishes an annotated image (with a circle around the marker) on a topic and saves the final frames. 

<table>
  <tr>
    <td>Gazebo Run</td>
     <td>Rviz panel</td>
  </tr>
  <tr>
    <td><img src="gazebo.png" width=533 height=300></td>
    <td><img src="rviz.png" width=533 height=300></td>
  </tr>
 </table>
---

# Description and features
- Gazebo simulation with a robot and 5 ArUco-marked boxes placed in a circular arrangement.
- ArUco detection pipeline (via `ros_aruco_opencv` package).
- `aruco_detections.py` node:
  - searches for all 5 marker IDs are detected and records their transformations w.r.t the world,
  - sorts IDs and selects the lowest remaining ID as target,
  - rotates the robot to center the marker in the camera image (angular visual servoing),
  - when centered, draws a circle on the image, publishes it on `/final_marker_image` and saves to disk,
  - repeats for each marker in ascending order until done.

---

# Prerequisites

OS: Ubuntu (tested on Ubuntu 22 for ROS 2 Humble and Ubuntu 24 for ROS 2 Jazzy).

ROS 2: Humble (works) and Jazzy (works) — you may need to clone the correct branch of ros_aruco_opencv for your distro.

Gazebo Harmonic is used — confirm the exact Gazebo version matching the ROS 2 distro (although Harmonic works on both ROS versions in this assignment).

Python: system Python3 (version used by ROS 2 distro; typically 3.10+).

System tools: colcon, vcstool (python3-vcstool), development libraries for ROS2 packages.

# Installation (bundle workspace)

This repository is provided as a bundle (multiple packages + repos files). The recommended workflow is to create a workspace and use vcs to import the referenced repositories.

```bash
# clone the assignment bundle (example)
git clone https://github.com/ExpRobLab/assignment1_bundle.git
cd assignment1_bundle

# create a workspace (if not using the repo's workspace layout)
mkdir -p ~/assignment_ws/src
cd ~/assignment_ws

# import repositories (if the bundle supplies .repos files inside the cloned repo)
# from within your workspace:
# vcs import src < path/to/assignment1_https.repos
# or, if using a local copy of the bundle where the .repos are available:
vcs import src < ./assignment1_bundle/assignment1_https.repos

# (Alternatively, manually clone the repo contents into ~/assignment_ws/src)

# build
colcon build 

# source the workspace 
source install/local_setup.bash

# launch the main demo:
ros2 launch assignment1 assignment.launch.py
# Or alternatively:
ros2 launch worlds_manager my_launch_assignment.py
```

NOTE: The repo references ros_aruco_opencv external package. Make sure that package is available in your src and that you check-out (or initially clone in vs) a branch compatible with your ROS 2 distro if necessary.

External dependencies (explicit):

The bundle depends on the ArUco OpenCV ROS package:

https://github.com/fictionlab/ros_aruco_opencv.git

and ROSBot for actual implementation:

https://husarion.com/manuals/rosbot/

If you import via .repos this will be pulled automatically the Jazzy version. If not, clone it into src/.

Important: the package maintainer may have a branch per ROS distro. If using Humble or Jazzy, check out the matching branch (or the aruco_detection branch referenced by your notes).

Install apt dependencies commonly required (replace <distro> where necessary; example for Humble):

```bash
sudo apt install ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport                  ros-humble-gazebo-ros-pkgs python3-opencv
```

# Launch files 

- `assignment1/launch/assignment.launch.py` — recommended main demo launch (spawns robot, relevant nodes and world).

# Nodes, topics & frames 

## Important nodes
- **aruco_detection_node** (script: `aruco_detections.py`) — core assignment logic.
- **aruco_tracker** (from `ros_aruco_opencv`) — publishes detections to `/aruco_detections`.

## Topics (publish / subscribe)

Subscribed:

- `/aruco_detections`
- `/camera/image/compressed`

Published:

- `/cmd_vel`
- `/final_marker_image`

## TF frames used

- `odom`
- `base_footprint`
- `marker_<ID>`

# Detailed explanation of aruco_detections.py logic

## Node lifecycle & subscriptions

Creates TF buffer, subscribes to detection and image topics, publishes velocity and images, uses a control loop.

## Detection Phase (and callback)

- Rotates robot while detecting
- For each detected marker:
  - TF lookup `odom -> marker_<id>`
  - Records transform + ID
- When all 5 detected:
  - Sorts IDs, selects lowest, enters "centering" state

## Control loop

- Active only in "centering"
- TF lookup: `base_footprint -> marker_<id>`
- Compute angle using atan2
- P-controller for angular velocity
- When centered:
  - publish stop
  - save + publish annotated image
  - state = "done"

## Image handling & annotation

- Converts compressed → OpenCV
- Draws circle on marker center
- Saves PNG
- Publishes annotated image

# Parameters & tuning

- `Kp = 1.0`
- `max_angular = 1.0`
- `threshold = 0.1 rad`
- `timer: 0.01s`

# Output files, RQt graphs & screenshots

- Output Images
<table>
  <tr>
    <td>Box 1</td>
    <td>Box 2</td>
    <td>Box 3</td>
  </tr>
  <tr>
    <td><img src="box1.png" width=533 height=300></td>
    <td><img src="box2.png" width=533 height=300></td>
    <td><img src="box3.png" width=533 height=300></td>
  </tr>
 </table>

- RQT Graph
<table>
  <tr>
    <td><img src="rqt.png" width=533 height=300></td>
  </tr>
 </table>

# Troubleshooting & tips

Camera topic mismatch:

```bash
ros2 topic list | grep image
```

# CLI snippets

```bash
ros2 node list
ros2 topic echo /aruco_detections
ros2 topic hz /camera/image/compressed
```
