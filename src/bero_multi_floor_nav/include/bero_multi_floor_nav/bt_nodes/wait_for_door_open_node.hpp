// wait_for_door_open_node.hpp

#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace bero_multi_floor_nav
{

class WaitForDoorOpenNode : public BT::StatefulActionNode
{
public:
  WaitForDoorOpenNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_sec", 60.0, "Timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void elevator_status_callback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr elevator_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr monitor_enable_pub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::atomic<bool> door_opened_{false};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace bero_multi_floor_nav
