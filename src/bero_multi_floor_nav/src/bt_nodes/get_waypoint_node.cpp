// get_waypoint_node.cpp

#include "bero_multi_floor_nav/bt_nodes/get_waypoint_node.hpp"

#include <algorithm>
#include <cctype>
#include <exception>

#include <yaml-cpp/yaml.h>

namespace bero_multi_floor_nav
{

std::map<std::string, geometry_msgs::msg::PoseStamped> GetWaypointNode::waypoints_;
bool GetWaypointNode::waypoints_loaded_ = false;

GetWaypointNode::GetWaypointNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw BT::RuntimeError("GetWaypointNode: missing required blackboard entry [node]");
  }
  
  if (!waypoints_loaded_) {
    load_waypoints();
    waypoints_loaded_ = true;
  }
}

// ========== Tick ==========

BT::NodeStatus GetWaypointNode::tick()
{
  // Input port을 통해 waypoint_name 조회
  auto waypoint_name_input = getInput<std::string>("waypoint_name");
  if (!waypoint_name_input) {
    RCLCPP_ERROR(node_->get_logger(), "[GetWaypointNode] Waypoint name not found");
    return BT::NodeStatus::FAILURE;
  }
  std::string waypoint_name = *waypoint_name_input;

  // waypoint_name 파싱
  std::string parsed_waypoint = parse_waypoint_name(waypoint_name);

  auto it = waypoints_.find(parsed_waypoint);
  if (it == waypoints_.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[GetWaypointNode] Waypoint '%s' (parsed: '%s') not found in loaded waypoints",
      waypoint_name.c_str(),
      parsed_waypoint.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Output port에 waypoint pose 설정
  setOutput("waypoint_pose", it->second);

  RCLCPP_INFO(
    node_->get_logger(),
    "[GetWaypointNode] Retrieved waypoint '%s': [%.2f, %.2f]",
    parsed_waypoint.c_str(),
    it->second.pose.position.x,
    it->second.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

// ========== Helper methods ==========

void GetWaypointNode::load_waypoints()
{
  try {
    if (!node_->has_parameter("multi_floor_waypoints_file_path")) {
      node_->declare_parameter<std::string>("multi_floor_waypoints_file_path", "");
    }

    std::string yaml_file;
    if (!node_->get_parameter("multi_floor_waypoints_file_path", yaml_file)) {
      throw BT::RuntimeError("GetWaypointNode Init Failed: 'multi_floor_waypoints_file_path' parameter not found");
    }

    YAML::Node config = YAML::LoadFile(yaml_file);
    if (!config["waypoints"]) {
      RCLCPP_ERROR(node_->get_logger(), "[GetWaypointNode] 'waypoints' key not found in %s", yaml_file.c_str());
      throw BT::RuntimeError("GetWaypointNode Init Failed: 'waypoints' key missing");
    }

    YAML::Node waypoints_node = config["waypoints"];
    for (YAML::const_iterator it = waypoints_node.begin(); it != waypoints_node.end(); ++it) {
      std::string name = it->first.as<std::string>();
      YAML::Node pose_node = it->second;

      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = pose_node["position"]["x"].as<double>();
      pose.pose.position.y = pose_node["position"]["y"].as<double>();
      pose.pose.position.z = pose_node["position"]["z"].as<double>();
      pose.pose.orientation.x = pose_node["orientation"]["x"].as<double>();
      pose.pose.orientation.y = pose_node["orientation"]["y"].as<double>();
      pose.pose.orientation.z = pose_node["orientation"]["z"].as<double>();
      pose.pose.orientation.w = pose_node["orientation"]["w"].as<double>();
      pose.header.frame_id = pose_node["frame_id"].as<std::string>();

      waypoints_[name] = pose;
    }
    
    RCLCPP_INFO(node_->get_logger(), "[GetWaypointNode] Loaded %lu waypoints from file", waypoints_.size());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "[GetWaypointNode] Failed to load waypoints: %s", e.what());
    throw BT::RuntimeError(std::string("GetWaypointNode Init Failed: ") + e.what());
  }
}

std::string GetWaypointNode::parse_waypoint_name(const std::string & waypoint_name)
{
  if (waypoint_name.length() == 4 &&
    std::all_of(waypoint_name.begin(), waypoint_name.end(), ::isdigit))
  {
    if (waypoint_name[1] == '2') {
      return waypoint_name.substr(1);
    } else {
      return waypoint_name.substr(2);
    }
  }
  return waypoint_name;
}

}  // namespace bero_multi_floor_nav
