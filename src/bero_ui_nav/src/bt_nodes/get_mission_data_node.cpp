// get_mission_data_node.cpp

#include "bero_ui_nav/bt_nodes/get_mission_data_node.hpp"

namespace bero_ui_nav
{

GetMissionDataNode::GetMissionDataNode(
  const std::string & service_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<bero_ui_nav::srv::GetMissionData>(service_name, conf)
{
}

// ========== Tick ==========

void GetMissionDataNode::on_tick()
{
  // 빈 요청
}

// ========== Halt ==========

void GetMissionDataNode::halt()
{
  nav2_behavior_tree::BtServiceNode<bero_ui_nav::srv::GetMissionData>::halt();
  RCLCPP_DEBUG(node_->get_logger(), "[GetMissionDataNode] halted");
}

// ========== Completion ==========

BT::NodeStatus GetMissionDataNode::on_completion(
  std::shared_ptr<bero_ui_nav::srv::GetMissionData::Response> response)
{
  if (response->success) {
    setOutput("target_room", response->target_room_number);
    setOutput("mission_uuid", response->mission_uuid);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bero_ui_nav
