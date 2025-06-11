cmake_minimum_required(VERSION 3.9)
project(nuturtlebot_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/WheelCommands.msg"
  DEPENDENCIES builtin_interfaces
)


ament_package()
