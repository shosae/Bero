// bt_nodes_plugin.cpp

#include "behaviortree_cpp_v3/bt_factory.h"

#include "bero_ui_nav/bt_nodes/get_waypoint_node.hpp"
#include "bero_ui_nav/bt_nodes/get_mission_data_node.hpp"
#include "bero_ui_nav/bt_nodes/wait_for_confirmation_node.hpp"
#include "bero_ui_nav/bt_nodes/phase_manager_node.hpp"

BT_REGISTER_NODES(factory)
{
  // *INDENT-OFF*
  BT::NodeBuilder builder_get_waypoint =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {return std::make_unique<bero_ui_nav::GetWaypointNode>(name, config);};

  BT::NodeBuilder builder_get_mission_data =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {return std::make_unique<bero_ui_nav::GetMissionDataNode>(name, config);};

  BT::NodeBuilder builder_wait_confirm =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {return std::make_unique<bero_ui_nav::WaitForConfirmationNode>(name, config);};

  BT::NodeBuilder builder_phase =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {return std::make_unique<bero_ui_nav::PhaseManagerNode>(name, config);};

  factory.registerBuilder<bero_ui_nav::GetWaypointNode>("GetWaypoint", builder_get_waypoint);
  factory.registerBuilder<bero_ui_nav::GetMissionDataNode>("GetMissionData", builder_get_mission_data);  // NOLINT
  factory.registerBuilder<bero_ui_nav::WaitForConfirmationNode>("WaitForConfirmation", builder_wait_confirm);  // NOLINT
  factory.registerBuilder<bero_ui_nav::PhaseManagerNode>("PhaseManager", builder_phase);
  // *INDENT-ON*
}
