// az_offset_calibrator_node.hpp

#pragma once

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace bero_multi_floor_nav
{

class AzOffsetCalibratorNode : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>
{
public:
  AzOffsetCalibratorNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
};

}  // namespace bero_multi_floor_nav
