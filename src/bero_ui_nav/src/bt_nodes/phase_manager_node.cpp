// phase_manager_node.cpp

#include "bero_ui_nav/bt_nodes/phase_manager_node.hpp"

namespace bero_ui_nav
{

PhaseManagerNode::PhaseManagerNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw BT::RuntimeError("PhaseManagerNode: missing required blackboard entry [node]");
  }
  // BT phase 업데이트 퍼블리셔
  bt_phase_pub_ = node_->create_publisher<std_msgs::msg::String>("/bt_phase", 10);
}

// ========== Tick ==========

BT::NodeStatus PhaseManagerNode::tick()
{
  std::string phase;
  if (!getInput("phase", phase)) {
    RCLCPP_ERROR(node_->get_logger(), "[PhaseManagerNode] Phase not found");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = phase;
  bt_phase_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[PhaseManagerNode] Published phase: %s", phase.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bero_ui_nav
