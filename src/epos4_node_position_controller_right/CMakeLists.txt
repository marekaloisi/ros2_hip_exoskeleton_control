cmake_minimum_required(VERSION 3.5)
project(epos4_dual_leg_manager)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GaitParams.msg"
  "msg/GaitFlag.msg"
  "msg/GaitDone.msg"
)

ament_package()
