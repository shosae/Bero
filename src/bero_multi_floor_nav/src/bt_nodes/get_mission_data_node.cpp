// get_mission_data_node.cpp

#include "bero_multi_floor_nav/bt_nodes/get_mission_data_node.hpp"

namespace bero_multi_floor_nav
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

  // 호실 번호 검증
  if (response->target_room_number.length() != 4) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[GetMissionDataNode] Invalid room number format: %s (must be 4 digits)",
      response->target_room_number.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Output ports 설정
  setOutput("target_room", response->target_room_number);
  setOutput("mission_uuid", response->mission_uuid);

  // 두 번째 자리에서 층수 추출
  std::string floor_str = response->target_room_number.substr(1, 1);
  setOutput("target_floor", floor_str);

  RCLCPP_INFO(
    node_->get_logger(),
    "[GetMissionDataNode] Retrieved mission data - Room: %s, Floor: %s",
    response->target_room_number.c_str(),
    floor_str.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bero_multi_floor_nav
