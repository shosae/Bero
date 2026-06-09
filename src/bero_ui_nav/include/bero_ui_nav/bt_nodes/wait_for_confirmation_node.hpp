// wait_for_confirmation_node.hpp

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "unique_identifier_msgs/msg/uuid.hpp"

namespace bero_ui_nav
{

class WaitForConfirmationNode : public BT::StatefulActionNode
{
public:
  WaitForConfirmationNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("confirm_topic", "Topic to wait for confirmation"),
      BT::InputPort<unique_identifier_msgs::msg::UUID>("mission_uuid", "Mission UUID to validate"),
      BT::InputPort<double>("timeout_sec", "Timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr confirm_callback_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> confirm_executor_;

  rclcpp::Subscription<unique_identifier_msgs::msg::UUID>::SharedPtr subscription_;

  void confirmation_callback(const unique_identifier_msgs::msg::UUID::SharedPtr msg);

  bool confirmation_received_{false};
  std::array<uint8_t, 16> expected_mission_id_{};
  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{300.0};
};

}  // namespace bero_ui_nav
