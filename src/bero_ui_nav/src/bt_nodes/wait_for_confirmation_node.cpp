// wait_for_confirmation_node.cpp

#include "bero_ui_nav/bt_nodes/wait_for_confirmation_node.hpp"

#include <algorithm>
#include <functional>

namespace bero_ui_nav
{

WaitForConfirmationNode::WaitForConfirmationNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw BT::RuntimeError("WaitForConfirmationNode: missing required blackboard entry [node]");
  }

  confirm_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false
  );

  confirm_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  confirm_executor_->add_callback_group(
    confirm_callback_group_,
    node_->get_node_base_interface()
  );
}

// ========== Lifecycle ==========

BT::NodeStatus WaitForConfirmationNode::onStart()
{
  // confirm topic 로드
  auto confirm_topic = getInput<std::string>("confirm_topic");
  if (!confirm_topic) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForConfirmationNode] Missing confirm_topic input");
    return BT::NodeStatus::FAILURE;
  }

  // mission UUID 로드
  auto mission_uuid = getInput<unique_identifier_msgs::msg::UUID>("mission_uuid");
  if (!mission_uuid) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForConfirmationNode] Missing mission_uuid input");
    return BT::NodeStatus::FAILURE;
  }
  expected_mission_id_ = mission_uuid->uuid;

  if (!subscription_) {
    rclcpp::QoS qos(1);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rclcpp::SubscriptionOptions options;
    options.callback_group = confirm_callback_group_;

    subscription_ = node_->create_subscription<unique_identifier_msgs::msg::UUID>(
      *confirm_topic,
      qos,
      std::bind(&WaitForConfirmationNode::confirmation_callback, this, std::placeholders::_1),
      options);
  }
  confirmation_received_ = false;

  auto timeout_input = getInput<double>("timeout_sec");
  timeout_sec_ = timeout_input ? *timeout_input : 300.0;
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    node_->get_logger(), "[WaitForConfirmationNode] Waiting on topic [%s] for mission UUID for [%.1f] seconds",  // NOLINT
    confirm_topic->c_str(),
    timeout_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForConfirmationNode::onRunning()
{
  confirm_executor_->spin_some();

  if (confirmation_received_) {
    RCLCPP_INFO(node_->get_logger(), "[WaitForConfirmationNode] Confirmation received");
    return BT::NodeStatus::SUCCESS;
  }

  // timeout 확인
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForConfirmationNode] timeout (%.1fsec), return to basecamp",
      timeout_sec_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForConfirmationNode::onHalted()
{
  RCLCPP_DEBUG(node_->get_logger(), "[WaitForConfirmationNode] Halted");
  confirmation_received_ = false;
  expected_mission_id_ = {};
  start_time_ = std::chrono::steady_clock::now();
}

// ========== Callbacks ==========

void WaitForConfirmationNode::confirmation_callback(
  const unique_identifier_msgs::msg::UUID::SharedPtr msg)
{
  // UUID 비교
  bool match = std::equal(msg->uuid.begin(), msg->uuid.end(), expected_mission_id_.begin());

  if (match) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[WaitForConfirmationNode] Valid confirmation received for current mission");
    confirmation_received_ = true;
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForConfirmationNode] Ignoring confirmation for different mission");
  }
}

}  // namespace bero_ui_nav
