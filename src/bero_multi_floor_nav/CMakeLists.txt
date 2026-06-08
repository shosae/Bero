cmake_minimum_required(VERSION 3.8)
project(bero_multi_floor_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(bero_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ========== Mission Manager Executable ==========

# Add mission manager as executable
add_executable(mission_manager
  src/mission_manager.cpp
)
target_link_libraries(mission_manager yaml-cpp)
ament_target_dependencies(mission_manager
  rclcpp
  rclcpp_action
  bero_msgs
  nav2_msgs
  geometry_msgs
  std_msgs
)

# Include directories for mission_manager
target_include_directories(mission_manager PRIVATE
  include
)

# ========== BT Plugin Library ==========

# Add BT plugin library
add_library(${PROJECT_NAME}_bt_nodes SHARED
  src/bt_nodes/get_mission_data_node.cpp
  src/bt_nodes/wait_for_confirmation_node.cpp
  src/bt_nodes/get_waypoint_node.cpp
  src/bt_nodes/phase_manager_node.cpp
  src/bt_nodes/press_button_node.cpp
  src/bt_nodes/wait_for_door_open_node.cpp
  src/bt_nodes/change_map_node.cpp
  src/bt_nodes/floor_estimation_node.cpp
  src/bt_nodes/az_offset_calibrator_node.cpp

  src/bt_nodes_plugin.cpp
)

target_compile_definitions(${PROJECT_NAME}_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(${PROJECT_NAME}_bt_nodes yaml-cpp)

ament_target_dependencies(${PROJECT_NAME}_bt_nodes
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  bero_msgs
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  nav2_msgs
  std_srvs
)

# Include directories for BT plugins
target_include_directories(${PROJECT_NAME}_bt_nodes PRIVATE
  include
)

# ========== Install ==========

# Install executables
install(TARGETS mission_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install BT plugin library
install(TARGETS ${PROJECT_NAME}_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install behavior trees
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install Python scripts
install(PROGRAMS
  scripts/mock_bt_ui.py
  scripts/mock_action_server.py
  scripts/mock_press_button_server.py
  scripts/mock_estimate_floor_server.py
  scripts/mock_az_offset_server.py
  scripts/mock_door_status_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Lint Tests ==========
# Lint tests (excluding copyright)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE "ament_cmake_copyright")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE "ament_cmake_cpplint")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE "ament_cmake_flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE "ament_cmake_uncrustify")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright" MAX_LINE_LENGTH 110)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(MAX_LINE_LENGTH 110)

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(MAX_LINE_LENGTH 110)
endif()

ament_package()
