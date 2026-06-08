// press_button_node.cpp

#include "bero_multi_floor_nav/bt_nodes/press_button_node.hpp"

namespace bero_multi_floor_nav
{

PressButtonNode::PressButtonNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<bero_msgs::action::PressButton>(xml_tag_name, "press_button", conf)
{
}

void PressButtonNode::on_tick()
{
  std::string button;
  if (!getInput("button", button)) {
    RCLCPP_ERROR(node_->get_logger(), "[PressButtonNode] Missing required input [button]");
    should_send_goal_ = false;
    return;
  }

  // 눌러야 할 button 정보 입력
  goal_.button = button;

  RCLCPP_INFO(node_->get_logger(), "[PressButtonNode] Sending goal for button: %s", goal_.button.c_str());
}

BT::NodeStatus PressButtonNode::on_success()
{
  if (result_.result->success) {
    RCLCPP_INFO(node_->get_logger(), "[PressButtonNode] SUCCESS - %s", result_.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PressButtonNode] FAILURE - %s", result_.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bero_multi_floor_nav
