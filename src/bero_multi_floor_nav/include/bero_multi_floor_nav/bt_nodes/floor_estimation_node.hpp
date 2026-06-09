// floor_estimation_node.hpp

#pragma once

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"

#include "bero_msgs/action/estimate_floor.hpp"

namespace bero_multi_floor_nav
{

class FloorEstimationNode : public nav2_behavior_tree::BtActionNode<bero_msgs::action::EstimateFloor>
{
public:
  using EstimateFloor = bero_msgs::action::EstimateFloor;

  FloorEstimationNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("start_floor", 0, "Current floor number"),
        BT::InputPort<int>("target_floor", 1, "Target floor number")
      });
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace bero_multi_floor_nav
