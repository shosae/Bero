cmake_minimum_required(VERSION 3.8)
project(bero_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DeliverToRoom.action"
  "action/EstimateFloor.action"
  "action/PressButton.action"
  "srv/GetMissionData.srv"
  "msg/DoorStateViz.msg"
  DEPENDENCIES unique_identifier_msgs std_msgs sensor_msgs
)

ament_package()
