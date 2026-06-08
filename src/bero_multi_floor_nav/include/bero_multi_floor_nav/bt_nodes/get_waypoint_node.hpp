// get_waypoint_node.hpp

#pragma once

#include <map>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bero_multi_floor_nav
{

class GetWaypointNode : public BT::SyncActionNode
{
public:
  GetWaypointNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("waypoint_name", "Name of the waypoint to get pose & orientation"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint_pose", "Waypoint's pose & orientation")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  static std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
  static bool waypoints_loaded_;
  void load_waypoints();

  std::string parse_waypoint_name(const std::string & waypoint_name);
};

}  // namespace bero_multi_floor_nav
