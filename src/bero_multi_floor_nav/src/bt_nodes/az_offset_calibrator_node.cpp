// az_offset_calibrator_node.cpp

#include "bero_multi_floor_nav/bt_nodes/az_offset_calibrator_node.hpp"

namespace bero_multi_floor_nav
{

AzOffsetCalibratorNode::AzOffsetCalibratorNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>(
    xml_tag_name,
    conf,
    "/calibrate_az_offset")
{
}

void AzOffsetCalibratorNode::on_tick()
{
  // 빈 요청
}

BT::NodeStatus AzOffsetCalibratorNode::on_completion(
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(), "[AzOffsetCalibratorNode] IMU Offset Calibration failed: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[AzOffsetCalibratorNode] IMU Offset Calibration successful: %s",
    response->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bero_multi_floor_nav
