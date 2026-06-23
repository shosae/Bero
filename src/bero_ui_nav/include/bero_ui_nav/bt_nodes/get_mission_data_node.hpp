// get_mission_data_node.hpp

#pragma once

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "bero_msgs/srv/get_mission_data.hpp"

namespace bero_ui_nav
{

class GetMissionDataNode : public nav2_behavior_tree::BtServiceNode<bero_msgs::srv::GetMissionData>
{
public:
  GetMissionDataNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<std::string>("target_room", "Target room number"),
        BT::OutputPort<unique_identifier_msgs::msg::UUID>("mission_uuid", "Mission UUID")
      });
  }

private:
  void on_tick() override;
  void halt() override;
  BT::NodeStatus on_completion(std::shared_ptr<bero_msgs::srv::GetMissionData::Response> response) override;
};

}  // namespace bero_ui_nav
