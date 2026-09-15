cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  CarAcceleration.msg
  CarMotor.msg
  CarTireInfo.msg
  CarVelocity.msg
  ControlCmd.msg
  StaPositionInfo.msg
  StateVector.msg
  TireInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
