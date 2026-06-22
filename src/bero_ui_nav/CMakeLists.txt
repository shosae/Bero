cmake_minimum_required(VERSION 3.8)
project(bero_ui_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(bero_msgs REQUIRED)

# ========== Mission Manager Executable ==========

# Add mission manager as executable
add_executable(mission_manager
  src/mission_manager.cpp
)
ament_target_dependencies(mission_manager
  ament_index_cpp
  rclcpp
  rclcpp_action
  bero_msgs
  nav2_msgs
  geometry_msgs
  std_msgs
)

# Include directories for mission_manager
target_include_directories(mission_manager PRIVATE include)

# ========== BT Plugin Library ==========

# Add BT plugin as shared library
add_library(${PROJECT_NAME}_bt_nodes SHARED
  src/bt_nodes/get_mission_data_node.cpp
  src/bt_nodes/wait_for_confirmation_node.cpp
  src/bt_nodes/get_waypoint_node.cpp
  src/bt_nodes/phase_manager_node.cpp
  src/bt_nodes_plugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}_bt_nodes
  rclcpp
  bero_msgs
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
)
target_compile_definitions(${PROJECT_NAME}_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# Include directories for BT plugins
target_include_directories(${PROJECT_NAME}_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========== Install ==========

# Install mission_manager
install(TARGETS mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install BT plugin
install(TARGETS ${PROJECT_NAME}_bt_nodes
  DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
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

# ========== Export ==========

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_bt_nodes)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  unique_identifier_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
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
