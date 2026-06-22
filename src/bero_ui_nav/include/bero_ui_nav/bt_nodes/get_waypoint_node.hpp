// get_waypoint_node.hpp

#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bero_ui_nav
{

class GetWaypointNode : public BT::SyncActionNode
{
public:
  GetWaypointNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("waypoint_name", "Name of the waypoint to get pose & orientation"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "waypoint_pose",
        "Waypoint's pose & orientation")
    };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::SyncParametersClient::SharedPtr param_client_;

  geometry_msgs::msg::PoseStamped get_waypoint_from_params(const std::string & waypoint_name);
};

}  // namespace bero_ui_nav
