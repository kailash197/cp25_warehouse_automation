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

class GLocalizeASClientNode : public rclcpp::Node {
public:
  using Localize = warehouse_automation_pkg::action::Localize;
  using GoalHandleLocalize = rclcpp_action::ClientGoalHandle<Localize>;

  GLocalizeASClientNode() : Node("client_glocalization_as_node") {
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/wh_glocalization_goal", 10,
        std::bind(&GLocalizeASClientNode::trigger_callback, this, _1));

    cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/wh_glocalization_cancel", 10,
        std::bind(&GLocalizeASClientNode::cancel_callback, this, _1));

    feedback_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/wh_glocalization_feedback", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/wh_glocalization_result", 10);

    action_client_ =
        rclcpp_action::create_client<Localize>(this, "/wh_global_localization");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp_action::Client<Localize>::SharedPtr action_client_;

  GoalHandleLocalize::SharedPtr
      current_goal_handle_; // to store active goal handle

  void trigger_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    RCLCPP_INFO(this->get_logger(),
                "Received trigger! Waiting for action server...");

    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }

    auto goal_msg = Localize::Goal(); // Empty goal
    RCLCPP_INFO(this->get_logger(), "Sending localization goal...");

    auto send_goal_options = rclcpp_action::Client<Localize>::SendGoalOptions();
    send_goal_options.feedback_callback =
        std::bind(&GLocalizeASClientNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GLocalizeASClientNode::result_callback, this, _1);
    send_goal_options.goal_response_callback =
        [this](GoalHandleLocalize::SharedPtr goal_handle) {
          if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            current_goal_handle_ = goal_handle;
          }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    if (!current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Received cancel trigger. Attempting to cancel goal...");

    auto cancel_future =
        action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();

    // auto result_code = rclcpp::spin_until_future_complete(
    //     this->get_node_base_interface(), cancel_future);

    // if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(this->get_logger(),
    //                  "Cancel request failed: could not contact action
    //                  server.");
    //     return;
    // }

    auto cancel_response = cancel_future.get();
    if (cancel_response->return_code ==
        action_msgs::srv::CancelGoal::Response::ERROR_NONE) {

      RCLCPP_INFO(this->get_logger(),
                  "Goal cancel request successfully accepted by server.");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Goal cancel failed with return code: %d ",
                  cancel_response->return_code);
    }
  }

  void
  feedback_callback(GoalHandleLocalize::SharedPtr,
                    const std::shared_ptr<const Localize::Feedback> feedback) {
    auto msg = std_msgs::msg::String();
    msg.data = "cov_x: " + std::to_string(feedback->cov_x) +
               ", cov_y: " + std::to_string(feedback->cov_y) +
               ", cov_yaw: " + std::to_string(feedback->cov_yaw) +
               ", status: " + feedback->status;

    feedback_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
  }

  void result_callback(const GoalHandleLocalize::WrappedResult &result) {
    auto msg = std_msgs::msg::String();
    msg.data =
        "success: " + std::string(result.result->success ? "true" : "false") +
        ", status: " + result.result->status;

    result_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());

    // Clear the handle since goal is done
    current_goal_handle_.reset();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GLocalizeASClientNode>());
  rclcpp::shutdown();
  return 0;
}
