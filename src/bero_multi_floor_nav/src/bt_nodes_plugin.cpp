// bt_nodes_plugin.cpp

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "bero_multi_floor_nav/bt_nodes/az_offset_calibrator_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/change_map_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/floor_estimation_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/get_mission_data_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/get_waypoint_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/phase_manager_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/press_button_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/wait_for_confirmation_node.hpp"
#include "bero_multi_floor_nav/bt_nodes/wait_for_door_open_node.hpp"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder_waypoint = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::GetWaypointNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::GetWaypointNode>("GetWaypoint", builder_waypoint);
  
  BT::NodeBuilder builder_get_mission = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::GetMissionDataNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::GetMissionDataNode>("GetMissionData", builder_get_mission);
  
  BT::NodeBuilder builder_wait_confirm = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::WaitForConfirmationNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::WaitForConfirmationNode>("WaitForConfirmation", builder_wait_confirm);
  
  BT::NodeBuilder builder_phase = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::PhaseManagerNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::PhaseManagerNode>("PhaseManager", builder_phase);

  BT::NodeBuilder builder_press_button = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::PressButtonNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::PressButtonNode>("PressButton", builder_press_button);

  BT::NodeBuilder builder_door_monitor = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::WaitForDoorOpenNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::WaitForDoorOpenNode>("WaitForDoorOpen", builder_door_monitor);

  BT::NodeBuilder builder_change_map = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::ChangeMapNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::ChangeMapNode>("ChangeMap", builder_change_map);

  BT::NodeBuilder builder_floor_estimation = 
    [](const std::string & name, const BT::NodeConfiguration & config) {  
      return std::make_unique<bero_multi_floor_nav::FloorEstimationNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::FloorEstimationNode>("FloorEstimation", builder_floor_estimation);

  BT::NodeBuilder builder_azcalibrator = 
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bero_multi_floor_nav::AzOffsetCalibratorNode>(name, config);};
  factory.registerBuilder<bero_multi_floor_nav::AzOffsetCalibratorNode>("AzOffsetCalibrator", builder_azcalibrator);

}
