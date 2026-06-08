// wait_for_door_open_node.cpp

#include "bero_multi_floor_nav/bt_nodes/wait_for_door_open_node.hpp"

#include <functional>
#include <memory>

namespace bero_multi_floor_nav
{

WaitForDoorOpenNode::WaitForDoorOpenNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw BT::RuntimeError("WaitForDoorOpenNode: missing required blackboard entry [node]");
  }

  // cb group 생성
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false
  );

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());

  // cb group 설정
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // door status 구독
  elevator_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/elevator/door_status",
    1,
    std::bind(&WaitForDoorOpenNode::elevator_status_callback, this, std::placeholders::_1),
    sub_opt
  );

  // door monitoring enable 발행
  monitor_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/elevator/door_monitoring_enable", 10
  );
}

BT::NodeStatus WaitForDoorOpenNode::onStart()
{
  start_time_ = std::chrono::steady_clock::now();

  // 이전 door_status 콜백을 한 번 처리하고, 이전 상태가 남지 않도록 플래그 초기화
  executor_->spin_some();
  door_opened_ = false;

  RCLCPP_INFO(node_->get_logger(), "[WaitForDoorOpenNode] Waiting for elevator door open");

  std_msgs::msg::Bool msg;
  msg.data = true;
  monitor_enable_pub_->publish(msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDoorOpenNode::onRunning()
{
  executor_->spin_some();

  auto timeout_input = getInput<double>("timeout_sec");
  double timeout_sec = timeout_input ? *timeout_input : 60.0;

  auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed > timeout_sec) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForDoorOpenNode] Timeout waiting for door open");

    std_msgs::msg::Bool msg;
    msg.data = false;
    monitor_enable_pub_->publish(msg);

    return BT::NodeStatus::FAILURE;
  }

  // 문 열림 체크
  if (door_opened_) {
    RCLCPP_INFO(node_->get_logger(), "[WaitForDoorOpenNode] Elevator door opening/opened");

    std_msgs::msg::Bool msg;
    msg.data = false;
    monitor_enable_pub_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForDoorOpenNode::onHalted()
{
  RCLCPP_DEBUG(node_->get_logger(), "[WaitForDoorOpenNode] Halted");

  // 모니터링 종료
  std_msgs::msg::Bool msg;
  msg.data = false;
  monitor_enable_pub_->publish(msg);
}

void WaitForDoorOpenNode::elevator_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string status = msg->data;
  // "opened" 또는 "opening" 상태 확인
  if (status.find("opened") != std::string::npos || status.find("opening") != std::string::npos) {
    door_opened_ = true;
  }
}
}  // namespace bero_multi_floor_nav
