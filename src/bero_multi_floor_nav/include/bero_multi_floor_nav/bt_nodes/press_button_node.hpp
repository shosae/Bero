// press_button_node.hpp

#pragma once

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"

#include "bero_msgs/action/press_button.hpp"

namespace bero_multi_floor_nav
{

class PressButtonNode : public nav2_behavior_tree::BtActionNode<bero_msgs::action::PressButton>
{
public:
  PressButtonNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("button", "Button to press")
      });
  }

private:
  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace bero_multi_floor_nav
