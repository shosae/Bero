// get_mission_data_node.cpp

#include "bero_ui_nav/bt_nodes/get_mission_data_node.hpp"

namespace bero_ui_nav
{

GetMissionDataNode::GetMissionDataNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<bero_msgs::srv::GetMissionData>(
    xml_tag_name,
    conf,
    "/get_mission_data")
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
  nav2_behavior_tree::BtServiceNode<bero_msgs::srv::GetMissionData>::halt();
  RCLCPP_DEBUG(node_->get_logger(), "[GetMissionDataNode] Halted");
}

// ========== Completion ==========

BT::NodeStatus GetMissionDataNode::on_completion(
  std::shared_ptr<bero_msgs::srv::GetMissionData::Response> response)
{
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "[GetMissionDataNode] Service returned failure");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("target_room", response->target_room_number);
  setOutput("mission_uuid", response->mission_uuid);

  RCLCPP_INFO(
    node_->get_logger(),
    "[GetMissionDataNode] Retrieved mission data - Room: %s",
    response->target_room_number.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bero_ui_nav
