// mission_manager.cpp

#include "bero_ui_nav/mission_manager.hpp"

#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bero_ui_nav
{

MissionManager::MissionManager(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  // ========== Worker Thread ==========

  mission_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false
  );

  deliver_to_room_action_server_ = rclcpp_action::create_server<DeliverToRoom>(
    this,
    "deliver_to_room",
    std::bind(&MissionManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MissionManager::handle_cancel, this, std::placeholders::_1),
    std::bind(&MissionManager::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    mission_callback_group_);

  mission_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  mission_executor_->add_callback_group(
    mission_callback_group_,
    this->get_node_base_interface()
  );

  executor_thread_ = std::thread([this] {mission_executor_->spin();});

  // ========== Main Thread ==========

  // Action Client(Nav2)
  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose"
  );

  const std::string package_dir = ament_index_cpp::get_package_share_directory("bero_ui_nav");
  single_floor_xml_path_ = package_dir + "/behavior_trees/single_floor_mission.xml";

  // Service Server(Get Mission Data)
  get_mission_data_service_server_ = this->create_service<bero_ui_nav::srv::GetMissionData>(
    "get_mission_data",
    std::bind(
      &MissionManager::handle_get_mission_data,
      this,
      std::placeholders::_1,
      std::placeholders::_2)
  );

  // Subscriber(BT phase)
  bt_phase_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/bt_phase",
    10,
    std::bind(&MissionManager::on_bt_phase, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Mission Manager started");
}

MissionManager::~MissionManager()
{
  RCLCPP_INFO(get_logger(), "Mission Manager shutting down...");
  cancel_current_nav2_goal();
  mission_executor_->cancel();
  executor_thread_.join();
  RCLCPP_INFO(get_logger(), "Mission Manager shutdown complete");
}

// ========== Action Server Callbacks ==========

rclcpp_action::GoalResponse MissionManager::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const DeliverToRoom::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received delivery request for room: %s", goal->room_number.c_str());

  if (get_mission_state().is_active) {
    RCLCPP_WARN(get_logger(), "Mission already running. Rejecting goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!waypoint_exists(goal->room_number)) {
    RCLCPP_ERROR(get_logger(), "Waypoint not found: %s", goal->room_number.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionManager::handle_cancel(
  const std::shared_ptr<GoalHandleDeliverToRoom>/*deliver_goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel delivery mission");
  cancel_current_nav2_goal();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionManager::handle_accepted(
  const std::shared_ptr<GoalHandleDeliverToRoom> deliver_goal_handle)
{
  RCLCPP_INFO(get_logger(), "Mission accepted, starting async execution");

  {
    std::lock_guard<std::mutex> lock(deliver_goal_mutex_);
    current_deliver_goal_handle_ = deliver_goal_handle;
  }

  const auto goal_uuid = deliver_goal_handle->get_goal_id();
  execute_deliver_mission(deliver_goal_handle, goal_uuid);
}

// ========== UI Mission Execution ==========

void MissionManager::execute_deliver_mission(
  const std::shared_ptr<GoalHandleDeliverToRoom> deliver_goal_handle,
  const rclcpp_action::GoalUUID & mission_uuid)
{
  const auto goal = deliver_goal_handle->get_goal();

  try {
    RCLCPP_INFO(get_logger(), "Initiating mission to room: %s", goal->room_number.c_str());
    set_mission_state(true, goal->room_number, mission_uuid);
    execute_custom_navigation_async(deliver_goal_handle, single_floor_xml_path_);
    RCLCPP_INFO(get_logger(), "Navigation request sent.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Mission failed: %s", e.what());
    auto result = make_result(false, "배달 실패: " + std::string(e.what()));
    deliver_goal_handle->abort(result);
    set_mission_state(false);
    clear_current_deliver_goal_handle();
  }
}

// ========== Nav2 Execution ==========

void MissionManager::execute_custom_navigation_async(
  const std::shared_ptr<GoalHandleDeliverToRoom> & deliver_goal_handle,
  const std::string & bt_xml_path)
{
  // nav2 서버 확인
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Navigation server unavailable");
    auto result = make_result(false, "내비게이션 서버가 준비되지 않았습니다.");
    deliver_goal_handle->abort(result);
    set_mission_state(false);
    clear_current_deliver_goal_handle();
    return;
  }

  // nav2 goal 설정
  auto nav2_goal = NavigateToPose::Goal();
  nav2_goal.pose = geometry_msgs::msg::PoseStamped();
  nav2_goal.behavior_tree = bt_xml_path;

  // nav2 goal options 설정
  auto nav2_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // options.response
  nav2_goal_options.goal_response_callback =
    [this, deliver_goal_handle](const GoalHandleNavigateToPose::SharedPtr & nav2_goal_handle) {
      if (!nav2_goal_handle) {
        RCLCPP_ERROR(get_logger(), "Navigation goal was rejected");
        auto result = make_result(false, "배달 요청이 거절되었습니다. 다시 시도해주세요.");
        deliver_goal_handle->abort(result);
        set_mission_state(false);
        clear_current_deliver_goal_handle();
      } else {
        RCLCPP_INFO(get_logger(), "Navigation goal accepted. Executing...");
        {
          std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
          current_nav2_goal_handle_ = nav2_goal_handle;
        }
      }
    };

  // options.result
  nav2_goal_options.result_callback =
    [this,
      deliver_goal_handle](const rclcpp_action::Client<NavigateToPose>::WrappedResult & nav2_result)
    {
      clear_current_nav2_goal();

      std::shared_ptr<DeliverToRoom::Result> result;

      switch (nav2_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Navigation SUCCEEDED");
          result = make_result(true, "배달 완료!");
          deliver_goal_handle->succeed(result);
          break;

        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "Navigation ABORTED");
          result = make_result(false, "오류로 인해 배달이 중단되었습니다.");
          deliver_goal_handle->abort(result);
          break;

        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "Navigation CANCELED");
          result = make_result(false, "배달이 취소되었습니다.");
          deliver_goal_handle->abort(result);
          break;

        default:
          RCLCPP_ERROR(get_logger(), "Navigation FAILED Unknown");
          result = make_result(false, "알 수 없는 이유로 배달이 중단되었습니다.");
          deliver_goal_handle->abort(result);
          break;
      }

      set_mission_state(false);
      clear_current_deliver_goal_handle();
      RCLCPP_INFO(get_logger(), "Mission ended. Ready for next.");
    };

  nav2_client_->async_send_goal(nav2_goal, nav2_goal_options);
}

// ========== Service Callback ==========

void MissionManager::handle_get_mission_data(
  const std::shared_ptr<bero_ui_nav::srv::GetMissionData::Request>/*request*/,
  std::shared_ptr<bero_ui_nav::srv::GetMissionData::Response> response)
{
  const auto mission = get_mission_state();

  RCLCPP_DEBUG(
    get_logger(),
    "[GetMissionData] called. is_active=%d target=%s",
    mission.is_active,
    mission.target_room_number.c_str());

  if (mission.is_active) {
    response->target_room_number = mission.target_room_number;
    std::copy(
      mission.mission_uuid.begin(), mission.mission_uuid.end(),
      response->mission_uuid.uuid.begin());
    response->success = true;
  } else {
    response->target_room_number.clear();
    response->success = false;
  }
}

// ========== Subscription Callback ==========

void MissionManager::on_bt_phase(const std_msgs::msg::String::SharedPtr msg)
{
  if (!get_mission_state().is_active) {
    return;
  }
  publish_phase_feedback(msg->data, describe_phase(msg->data));
}

std::string MissionManager::describe_phase(const std::string & phase) const
{
  std::unordered_map<std::string, std::string> descriptions = {
    {"moving_to_room", "호실로 배달 중... "},
    {"confirm", "배달 완료!\n수령 후 확인 버튼을 눌러주세요."},
    {"return", "복귀 중..."},
  };

  const auto mission = get_mission_state();
  if (!mission.target_room_number.empty()) {
    descriptions["moving_to_room"] = mission.target_room_number + "호실로 배달 중... ";
  }

  const auto it = descriptions.find(phase);
  if (it != descriptions.end()) {
    return it->second;
  }
  return "배달 진행 중... ";
}

void MissionManager::publish_phase_feedback(
  const std::string & phase,
  const std::string & status)
{
  std::shared_ptr<GoalHandleDeliverToRoom> deliver_goal_handle;
  {
    std::lock_guard<std::mutex> lock(deliver_goal_mutex_);
    deliver_goal_handle = current_deliver_goal_handle_.lock();
  }

  if (deliver_goal_handle) {
    auto feedback = std::make_shared<DeliverToRoom::Feedback>();
    feedback->phase = phase;
    feedback->status = status;
    deliver_goal_handle->publish_feedback(feedback);
  }
}

// ========== Helper Functions ==========

bool MissionManager::waypoint_exists(const std::string & room_number) const
{
  const std::string param_key = "waypoints." + room_number + ".position.x";
  return this->has_parameter(param_key);
}

std::shared_ptr<MissionManager::DeliverToRoom::Result> MissionManager::make_result(
  bool success,
  const std::string & message) const
{
  auto result = std::make_shared<MissionManager::DeliverToRoom::Result>();
  result->success = success;
  result->message = message;
  return result;
}

void MissionManager::set_mission_state(
  bool active,
  const std::string & room_number,
  const rclcpp_action::GoalUUID & mission_uuid)
{
  std::lock_guard<std::mutex> lock(mission_mutex_);
  current_mission_.is_active = active;
  if (active) {
    current_mission_.target_room_number = room_number;
    current_mission_.mission_uuid = mission_uuid;
  } else {
    current_mission_.target_room_number.clear();
    current_mission_.mission_uuid.fill(0);
  }
}

MissionManager::MissionData MissionManager::get_mission_state() const
{
  std::lock_guard<std::mutex> lock(mission_mutex_);
  return current_mission_;
}

void MissionManager::cancel_current_nav2_goal()
{
  GoalHandleNavigateToPose::SharedPtr nav2_goal_handle;
  {
    std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
    nav2_goal_handle = current_nav2_goal_handle_.lock();
  }

  if (nav2_goal_handle) {
    nav2_client_->async_cancel_goal(nav2_goal_handle);
  }
}

void MissionManager::clear_current_nav2_goal()
{
  std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
  current_nav2_goal_handle_.reset();
}

void MissionManager::clear_current_deliver_goal_handle()
{
  std::lock_guard<std::mutex> lock(deliver_goal_mutex_);
  current_deliver_goal_handle_.reset();
}

}  // namespace bero_ui_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options = options
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<bero_ui_nav::MissionManager>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
