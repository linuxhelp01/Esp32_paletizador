cmake_minimum_required(VERSION 3.8)
project(palletizer_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveXYZ.action"
  "action/HomeAxis.action"
  "action/GoOrigin.action"
  "srv/EnableAxis.srv"
  "srv/SetZero.srv"
  "srv/SetAxisLimits.srv"
  "srv/ClearFault.srv"
  "srv/ReleaseStall.srv"
  "srv/GetDriverStatus.srv"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
