// change_map_node.cpp

#include "bero_multi_floor_nav/bt_nodes/change_map_node.hpp"

#include <cstddef>

namespace bero_multi_floor_nav
{

ChangeMapNode::ChangeMapNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::LoadMap>(xml_tag_name, conf, "/map_server/load_map")
{
  // floor별 map path 초기화
  if (!node_->has_parameter("multi_floor_map_base_path")) {
    node_->declare_parameter<std::string>("multi_floor_map_base_path", "");
  }

  std::string map_base_path;
  if (!node_->get_parameter("multi_floor_map_base_path", map_base_path)) {
    throw BT::RuntimeError("ChangeMapNode Init Failed: 'multi_floor_map_base_path' parameter not found");
  }

  map_paths_[0] = map_base_path + "L.yaml";
  map_paths_[1] = map_base_path + "L.yaml";
  map_paths_[2] = map_base_path + "2.yaml";
  map_paths_[3] = map_base_path + "upper.yaml";

  // initialpose 퍼블리셔 생성
  initialpose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
}

void ChangeMapNode::on_tick()
{
  // InputPort로부터 층 정보 얻기
  int floor;
  if (!getInput("floor", floor)) {
    RCLCPP_ERROR(node_->get_logger(), "[ChangeMapNode] Missing required input [floor]");
    should_send_request_ = false;
    return;
  }

  // 맵 파일 경로 검색, 3층 이상은 동일 맵
  std::string map_path;
  if (floor >= 3) {
    auto it = map_paths_.find(3);
    if (it != map_paths_.end()) {
      map_path = it->second;
    }
  } else {
    auto it = map_paths_.find(floor);
    if (it != map_paths_.end()) {
      map_path = it->second;
    }
  }

  if (map_path.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[ChangeMapNode] Map path not found for floor %d", floor);
    should_send_request_ = false;
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[ChangeMapNode] Changing map to floor %d: %s", floor, map_path.c_str());
  
  request_->map_url = map_path;
}

BT::NodeStatus ChangeMapNode::on_completion(std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "[ChangeMapNode] Map loading failed");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[ChangeMapNode] Map loaded successfully");

  geometry_msgs::msg::PoseWithCovarianceStamped init_msg;
  init_msg.header.stamp = node_->now();
  init_msg.header.frame_id = "map";

  // InputPort로부터 초기 위치 받기
  geometry_msgs::msg::PoseStamped input_pose;
  if (getInput("initial_pose", input_pose)) {
    init_msg.pose.pose = input_pose.pose;
    RCLCPP_INFO(node_->get_logger(), "[ChangeMapNode] Setting initial pose: [%.2f, %.2f]",
      input_pose.pose.position.x, input_pose.pose.position.y);
  } else {
    init_msg.pose.pose.position.x = 0.0;
    init_msg.pose.pose.position.y = 0.0;
    init_msg.pose.pose.position.z = 0.0;
    init_msg.pose.pose.orientation.w = 1.0;
    RCLCPP_WARN(node_->get_logger(), "[ChangeMapNode] Initial pose input not found, using default (0,0,0)");
  }

  // 공분산 설정
  for (size_t i = 0; i < 36; ++i) {
    init_msg.pose.covariance[i] = 0.0;
  }
  init_msg.pose.covariance[0] = 0.09;  // x (약 30cm)
  init_msg.pose.covariance[7] = 0.09;  // y (약 30cm)
  init_msg.pose.covariance[35] = 0.068; // yaw (약 15도)

  initialpose_pub_->publish(init_msg);
  RCLCPP_INFO(node_->get_logger(), "[ChangeMapNode] Initial pose published successfully");
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bero_multi_floor_nav
