# Visual-Pose-Graph-Estimation
Final project for MSAI495: Computer Vision


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


## TODOs
- Setup turtlebot in ROS2 with camera (Intel Realsense, Zedd 2i)
- Research pose-graph estimation using visual sensor

## Dependency install:
- sudo apt-get install ros-jazzy-libg2o


## Resource
- https://nu-msr.github.io/ros_notes/ros2/computer_vision.html
- https://www.mathworks.com/help/nav/ug/reduce-drift-visual-odom-pose-graph.html
- https://youtube.com/playlist?list=PLoAomNxPy04feqEsgs2oTGrEFztkSoLjH&si=HzNw6oAGWwe0BXag
- https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_setup.html

## Hardware:
- TurtleBot3
  [Static IP: 192.168.2.1 | Name: Teelte]
- RealSense D435I
- LiPo Battery
