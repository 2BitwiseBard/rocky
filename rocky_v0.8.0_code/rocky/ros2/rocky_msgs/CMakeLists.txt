cmake_minimum_required(VERSION 3.16)
project(rocky_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GaitCommand.msg"
  "msg/BodyPoseCommand.msg"
  "msg/HandCommand.msg"
  "msg/ContactState.msg"
  "msg/ServoHealth.msg"
  DEPENDENCIES std_msgs
)

ament_package()
