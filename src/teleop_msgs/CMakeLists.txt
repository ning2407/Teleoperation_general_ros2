cmake_minimum_required(VERSION 3.8)
project(teleop_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndEffectorState.msg"
  "msg/GripperCommand.msg"
  "msg/GripperState.msg"
  "msg/RobotStatus.msg"
  "msg/SensorState.msg"
  "msg/TeleopCommand.msg"
  "msg/TeleopObservation.msg"
  "msg/TeleopStatus.msg"
  "srv/ResetEmergencyStop.srv"
  "srv/SetControlMode.srv"
  "srv/SetTeleopEnable.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
