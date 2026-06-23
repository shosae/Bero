// get_waypoint_node.cpp

#include "bero_ui_nav/bt_nodes/get_waypoint_node.hpp"

#include <array>

namespace bero_ui_nav
{

GetWaypointNode::GetWaypointNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw BT::RuntimeError("GetWaypointNode: missing required blackboard entry [node]");
  }
  // mission_manager 노드의 파라미터에 접근
  param_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "mission_manager");
}

// ========== Tick ==========

BT::NodeStatus GetWaypointNode::tick()
{
  // Input port을 통해 room number 조회
  auto waypoint_name_input = getInput<std::string>("waypoint_name");
  if (!waypoint_name_input) {
    RCLCPP_ERROR(node_->get_logger(), "[GetWaypointNode] Waypoint name not found");
    return BT::NodeStatus::FAILURE;
  }
  std::string waypoint_name = *waypoint_name_input;

  // waypoint_name Parameter에서 waypoint pose 조회
  auto waypoint_pose = get_waypoint_from_params(waypoint_name);
  if (waypoint_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[GetWaypointNode] Waypoint '%s' not found in parameters",
      waypoint_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Output port에 waypoint pose 설정
  setOutput("waypoint_pose", waypoint_pose);

  RCLCPP_INFO(
    node_->get_logger(),
    "[GetWaypointNode] Retrieved waypoint '%s': [%.2f, %.2f]",
    waypoint_name.c_str(),
    waypoint_pose.pose.position.x,
    waypoint_pose.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

// ========== Helper methods ==========

geometry_msgs::msg::PoseStamped GetWaypointNode::get_waypoint_from_params(
  const std::string & waypoint_name)
{
  geometry_msgs::msg::PoseStamped pose;

  try {
    std::string prefix = "waypoints." + waypoint_name;

    static const std::array<const char *, 8> key_suffixes = {
      ".position.x", ".position.y", ".position.z",
      ".orientation.x", ".orientation.y",
      ".orientation.z", ".orientation.w",
      ".frame_id"
    };
    std::vector<std::string> keys;
    keys.reserve(key_suffixes.size());  // 메모리 할당 최적화
    for (const char * suffix : key_suffixes) {
      keys.push_back(prefix + suffix);
    }

    auto params = param_client_->get_parameters(keys);

    if (params.size() != keys.size()) {
      return geometry_msgs::msg::PoseStamped{};
    }

    pose.pose.position.x = params[0].as_double();
    pose.pose.position.y = params[1].as_double();
    pose.pose.position.z = params[2].as_double();
    pose.pose.orientation.x = params[3].as_double();
    pose.pose.orientation.y = params[4].as_double();
    pose.pose.orientation.z = params[5].as_double();
    pose.pose.orientation.w = params[6].as_double();
    pose.header.frame_id = params[7].as_string();

    return pose;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[GetWaypointNode] Error retrieving waypoint '%s': %s",
      waypoint_name.c_str(),
      e.what());
  }
  return geometry_msgs::msg::PoseStamped();
}

}  // namespace bero_ui_nav
