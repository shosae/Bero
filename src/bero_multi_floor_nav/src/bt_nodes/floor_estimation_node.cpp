// floor_estimation_node.cpp

#include "bero_multi_floor_nav/bt_nodes/floor_estimation_node.hpp"

namespace bero_multi_floor_nav
{

FloorEstimationNode::FloorEstimationNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<bero_msgs::action::EstimateFloor>(xml_tag_name, "estimate_floor", conf)
{
}

void FloorEstimationNode::on_tick()
{
  int start_floor;
  int target_floor;

  if (!getInput("start_floor", start_floor)) {
    start_floor = 0;
  }
  if (!getInput("target_floor", target_floor)) {
    target_floor = 1;
  }

  // 시작 층과 목표 층 지정
  goal_.start_floor = start_floor;
  goal_.target_floor = target_floor;

    RCLCPP_INFO(
    node_->get_logger(),
    "[FloorEstimationNode] Sending goal: %d -> %d",
    start_floor, target_floor);
}

BT::NodeStatus FloorEstimationNode::on_success()
{
  if (result_.result->success) {
    RCLCPP_INFO(node_->get_logger(), "[FloorEstimationNode] SUCCESS - %s", result_.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[FloorEstimationNode] FAILURE - %s", result_.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus FloorEstimationNode::on_aborted()
{
  RCLCPP_ERROR(node_->get_logger(), "[FloorEstimationNode] Action ABORTED");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FloorEstimationNode::on_cancelled()
{
  RCLCPP_WARN(node_->get_logger(), "[FloorEstimationNode] Action CANCELLED");
  return BT::NodeStatus::FAILURE;
}

}  // namespace bero_multi_floor_nav
