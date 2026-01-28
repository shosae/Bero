// phase_manager_node.hpp

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "std_msgs/msg/string.hpp"

namespace bero_ui_nav
{

class PhaseManagerNode : public BT::SyncActionNode
{
public:
  PhaseManagerNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("phase", "Phase to change ui")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_phase_pub_;
};

}  // namespace bero_ui_nav
