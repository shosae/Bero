// change_map_node.hpp

#pragma once

#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bero_multi_floor_nav
{

class ChangeMapNode : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::LoadMap>
{
public:
  ChangeMapNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("floor", "Target floor number for map switch"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "initial_pose",
          "Initial pose to publish after map switch")
      });
  }

  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) override;

private:
  std::map<int, std::string> map_paths_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
};

}  // namespace bero_multi_floor_nav
