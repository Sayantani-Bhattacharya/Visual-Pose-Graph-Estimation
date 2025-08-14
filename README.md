# Real-Time Visual Pose-Graph-Estimation
### Final project for MSAI495: Computer Vision
#### Author: Sayantani Bhattacharya, Sharwin Patil 

We implemented a ROS2 Package that extracts synchronized stereo or monocular raw image frames from an Intel RealSense Camera. Upon initialization, the node configures itself based on runtime parameters including camera mode (stereo/monocular), feature extraction method (ORB/SIFT), and optimization frequency. For stereo operation, it establishes synchronized subscribers for both infrared cameras and their calibration data, while monocular mode uses the color camera stream. For more details visit [Portfolio Post](https://sayantani-bhattacharya.github.io/posts/vio/)

And extended the project to develop a F1 Ghost Car Racing using Visual-Inertial Odometry. A Multi-player game that scores players on how accurate a defined race track is followed. [Game Demo](https://sayantani-bhattacharya.github.io/posts/f_one/)

## Quickstart for TurtleBot3:
1. Connect to NUMSR (Automatic no Static IP)
   
2.      ssh -oSendEnv=30 msr@teetle #NAME OF TURTLEBOT GOES HERE.
        # Password is robotics!

3. Verify working status: should hear jingle  ```opencr_install_firmware```
   
4.     # Not sure if needed
       source /opt/ros/iron/setup.bash
       source ~/ros_home_ws/install/setup.bash
       
5.     # Run the setup node: 
        source /home/msr/install/setup.bash
        ros2 run numsr_turtlebot numsr_turtlebot
   
6.     # Test for wheel command to Move forward: 
        ros2 topic pub /wheel_cmd nuturtlebot_msgs/msg/WheelCommands "{left_velocity: 100, right_velocity: 100}" 

## Dependency Installation:
- sudo apt-get install ros-jazzy-libg2o
- https://www.stereolabs.com/docs/installation/linux : Install Zed SDK
- https://github.com/stereolabs/zed-ros2-wrapper?tab=readme-ov-file : Install ros2 zed wrapper

## Resource
- https://nu-msr.github.io/ros_notes/ros2/computer_vision.html
- https://www.mathworks.com/help/nav/ug/reduce-drift-visual-odom-pose-graph.html
- https://youtube.com/playlist?list=PLoAomNxPy04feqEsgs2oTGrEFztkSoLjH&si=HzNw6oAGWwe0BXag
- https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_setup.html
- https://stackoverflow.com/questions/13376441/relative-camera-pose-estimation-using-opencv
- https://docs.ros.org/en/ros2_packages/jazzy/api/message_filters/doc/Tutorials/Writing-A-Time-Synchronizer-Cpp.html
- https://docs.ros.org/en/ros2_packages/jazzy/api/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html

## Hardware:
- TurtleBot3
  [Static IP: 192.168.2.1 | Name: Teelte]
- RealSense D435I
- Zed 2i
- LiPo Battery
