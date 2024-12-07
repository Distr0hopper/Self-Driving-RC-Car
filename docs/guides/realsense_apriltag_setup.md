# RealSense and AprilTag Setup Guide

## RealSense Installation

1. Install RealSense SDK:
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC044F831AC80A06380C8B3A55A6F3EFCDE
   sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
   sudo apt install librealsense2-dkms librealsense2-utils

2. Test Camera:
   realsense-viewer

## ROS2 Setup

1. Install Required Packages:
   sudo apt install ros-humble-realsense2-camera
   sudo apt install ros-humble-apriltag
   sudo apt install ros-humble-apriltag-ros
   sudo apt install ros-humble-image-proc
   sudo apt install ros-humble-rqt*

2. Launch Camera:
   ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true

## AprilTag Detection

1. Workspace Setup:
   cd ~/kimono_ws
   colcon build
   source install/setup.bash

2. Launch Detection:
   ros2 launch realsense_apriltag rs_tag_detection.launch.py

3. Verify Detection:
   ros2 topic list | grep -E 'camera|tag'
   ros2 topic echo /apriltag/detections
   ros2 topic echo /tag_detections

4. Visualization:
   source /opt/ros/humble/setup.bash
   ros2 run rqt_image_view rqt_image_view

## Configuration and Troubleshooting

1. View/Edit Launch File:
   gedit ~/kimono_ws/src/realsense_apriltag/launch/rs_tag_detection.launch.py

2. Rebuild Specific Packages:
   cd ~/kimono_ws
   colcon build --packages-select realsense_apriltag
   source install/setup.bash

3. Common Issues:
   - If detection isn't working, check camera topics are publishing
   - Verify AprilTag is within camera's field of view
   - Check lighting conditions
   - Ensure correct tag family is configured in launch file

## Usage Tips

1. Monitor System:
   - Use rqt_image_view to verify camera feed
   - Check topic output to confirm detections
   - Monitor transform broadcasts if using for localization

2. Best Practices:
   - Always source workspace after rebuilding
   - Test basic camera functionality before AprilTag detection
   - Keep AprilTag perpendicular to camera when possible
   - Maintain good lighting conditions
