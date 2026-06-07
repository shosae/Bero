// mission_manager.hpp

#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"

#include "bero_msgs/action/deliver_to_room.hpp"
#include "bero_msgs/srv/get_mission_data.hpp"

namespace bero_multi_floor_nav
{
class MissionManager : public rclcpp::Node
{
public:
  using DeliverToRoom = bero_msgs::action::DeliverToRoom;
  using GoalHandleDeliverToRoom = rclcpp_action::ServerGoalHandle<DeliverToRoom>;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MissionManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MissionManager() override;

private:
  // ========== ROS Interfaces ==========
  // Action server for /deliver_to_room
  rclcpp_action::Server<DeliverToRoom>::SharedPtr deliver_to_room_action_server_;

  // Service server for /get_mission_data
  rclcpp::Service<bero_msgs::srv::GetMissionData>::SharedPtr get_mission_data_service_server_;

  // Action client for /navigate_to_pose(Nav2)
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

  // Subscriber for BT phase messages
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_phase_sub_;

  // ========== DeliverToRoom goal handle tracking ==========
  std::mutex deliver_goal_mutex_;
  std::weak_ptr<GoalHandleDeliverToRoom> current_deliver_goal_handle_;

  // ========== Nav2 goal handle tracking ==========
  std::mutex nav2_goal_mutex_;
  std::weak_ptr<GoalHandleNavigateToPose> current_nav2_goal_handle_;

  // ========== Mission state ==========
  mutable std::mutex mission_mutex_;

  struct MissionData
  {
    std::string target_room_number = "";
    rclcpp_action::GoalUUID mission_uuid;
    bool is_active = false;
  } current_mission_;

  // Worker thread
  rclcpp::CallbackGroup::SharedPtr mission_callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr mission_executor_;
  std::thread executor_thread_;

  // Cached paths
  std::string multi_floor_mission_xml_path_;

  // Room validation
  std::set<int> valid_floors_;
  std::set<std::string> valid_rooms_;
  void load_validation_data(const std::string & yaml_path);

  // ========== Action server callbacks ==========
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DeliverToRoom::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDeliverToRoom> deliver_goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleDeliverToRoom> deliver_goal_handle);

  void execute_deliver_mission(
    const std::shared_ptr<GoalHandleDeliverToRoom> deliver_goal_handle,
    const rclcpp_action::GoalUUID & mission_uuid);

  void execute_custom_navigation_async(
    const std::shared_ptr<GoalHandleDeliverToRoom> & goal_handle,
    const std::string & bt_xml_path);

  // ========== Service callback ==========
  void handle_get_mission_data(
    const std::shared_ptr<bero_msgs::srv::GetMissionData::Request> request,
    std::shared_ptr<bero_msgs::srv::GetMissionData::Response> response);

  // ========== Subscription Callback ==========
  void on_bt_phase(const std_msgs::msg::String::SharedPtr msg);
  std::string describe_phase(const std::string & phase) const;
  void publish_phase_feedback(const std::string & phase, const std::string & status);

  // ========== Helper Functions ==========
  std::shared_ptr<DeliverToRoom::Result> make_result(
    bool success,
    const std::string & message) const;

  void set_mission_state(
    bool active,
    const std::string & room_number = "",
    const rclcpp_action::GoalUUID & mission_uuid = rclcpp_action::GoalUUID());
  MissionData get_mission_state() const;

  bool is_goal_valid(const std::string & room_number) const;
  void cancel_current_nav2_goal();
  void clear_current_nav2_goal();
  void clear_current_deliver_goal_handle();
};

}  // namespace bero_multi_floor_nav
