#include <chrono>
#include <memory>
#include <string>

#include "action_msgs/srv/cancel_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "warehouse_automation_pkg/action/localize.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class GlobalLocalizationClient : public rclcpp::Node {
public:
  using Localize = warehouse_automation_pkg::action::Localize;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Localize>;

  explicit GlobalLocalizationClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("global_localization_client", options) {
    // Initialize publishers and subscribers
    initialize_components();

    // Create action client
    action_client_ =
        rclcpp_action::create_client<Localize>(this, "/wh_global_localization");
  }

private:
  void initialize_components() {
    // Subscribers
    trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/wh_glocalization_goal", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) { send_goal(); });

    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/wh_glocalization_cancel", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          cancel_active_goal();
        });

    // Publishers
    feedback_pub_ = create_publisher<std_msgs::msg::String>(
        "/wh_glocalization_feedback", 10);
    result_pub_ =
        create_publisher<std_msgs::msg::String>("/wh_glocalization_result", 10);
  }

  void send_goal() {
    RCLCPP_INFO(get_logger(), "Received localization trigger");

    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return;
    }

    auto goal = Localize::Goal();
    RCLCPP_INFO(get_logger(), "Sending localization goal");

    auto send_goal_options = rclcpp_action::Client<Localize>::SendGoalOptions();
    configure_goal_options(send_goal_options);

    auto future = action_client_->async_send_goal(goal, send_goal_options);

    // Handle goal acceptance asynchronously
    future.then([this](std::shared_future<GoalHandle::SharedPtr> future) {
      if (const auto goal_handle = future.get()) {
        current_goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Goal accepted by server");
      } else {
        RCLCPP_WARN(get_logger(), "Goal rejected by server");
      }
    });
  }

  void configure_goal_options(
      rclcpp_action::Client<Localize>::SendGoalOptions &options) {
    options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const Localize::Feedback> feedback) {
          handle_feedback(feedback);
        };

    options.result_callback = [this](const GoalHandle::WrappedResult &result) {
      handle_result(result);
    };
  }

  void cancel_active_goal() {
    if (!current_goal_handle_) {
      RCLCPP_WARN(get_logger(), "No active goal to cancel");
      return;
    }

    RCLCPP_INFO(get_logger(), "Requesting goal cancellation");

    auto cancel_future =
        action_client_->async_cancel_goal(current_goal_handle_);

    cancel_future.then([this](std::shared_future<
                              action_msgs::srv::CancelGoal::Response::SharedPtr>
                                  future) {
      const auto response = future.get();

      if (response->return_code ==
          action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
        RCLCPP_INFO(get_logger(), "Goal cancellation accepted by server");
      } else {
        RCLCPP_WARN(get_logger(), "Goal cancellation failed with code: %d",
                    response->return_code);
      }

      current_goal_handle_.reset();
    });
  }

  void
  handle_feedback(const std::shared_ptr<const Localize::Feedback> feedback) {
    std_msgs::msg::String msg;
    msg.data = fmt::format("cov_x: {}, cov_y: {}, cov_yaw: {}, status: {}",
                           feedback->cov_x, feedback->cov_y, feedback->cov_yaw,
                           feedback->status);

    feedback_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Feedback: %s", msg.data.c_str());
  }

  void handle_result(const GoalHandle::WrappedResult &result) {
    std_msgs::msg::String msg;
    msg.data = fmt::format("success: {}, status: {}",
                           result.result->success ? "true" : "false",
                           result.result->status);

    result_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Result: %s", msg.data.c_str());

    current_goal_handle_.reset();
  }

  // Member variables
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp_action::Client<Localize>::SharedPtr action_client_;
  GoalHandle::SharedPtr current_goal_handle_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalizationClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}