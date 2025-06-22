#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "warehouse_automation_pkg/action/localize.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using Localize = warehouse_automation_pkg::action::Localize;
using GoalHandleLocalize = rclcpp_action::ServerGoalHandle<Localize>;
using GoalHandleLocalize =
    rclcpp_action::ServerGoalHandle<warehouse_automation_pkg::action::Localize>;

class GlobalLocalization
    : public rclcpp::Node,
      public std::enable_shared_from_this<GlobalLocalization> {
public:
  GlobalLocalization() : Node("global_localization_node") {

    env = this->declare_parameter<std::string>("env", "sim");
    std::string topic = (env == "sim")
                            ? "/diffbot_base_controller/cmd_vel_unstamped"
                            : "/cmd_vel";
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

    client_ = this->create_client<std_srvs::srv::Empty>(
        "/reinitialize_global_localization");

    amcl_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          cov_ = msg->pose.covariance;
          amcl_updated_ = true;
        });

    timer_ = nullptr;

    action_server_ = rclcpp_action::create_server<Localize>(
        this, "wh_global_localization",
        std::bind(&GlobalLocalization::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GlobalLocalization::handleCancel, this,
                  std::placeholders::_1),
        std::bind(&GlobalLocalization::handleAccepted, this,
                  std::placeholders::_1));
    fake_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10); // Standard topic AMCL listens to

    RCLCPP_INFO(this->get_logger(),
                "WH Global Localization action server ready.");
  }

private:
  std::string env;
  std::shared_ptr<GoalHandleLocalize> active_goal_;
  rclcpp_action::Server<Localize>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<double, 36> cov_;
  bool amcl_updated_ = false;
  int counter_ = 0;
  //   GoalHandleLocalize::SharedPtr active_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      fake_pose_pub_;

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const Localize::Goal>) {
    RCLCPP_INFO(this->get_logger(), "Received localization goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleLocalize>) {
    RCLCPP_INFO(this->get_logger(), "Localization goal canceled.");
    if (timer_)
      timer_->cancel();
    publishTwist(0.0);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleLocalize> goal_handle) {
    active_goal_ = goal_handle;
    std::thread([this]() { this->startLocalization(); }).detach();
  }

  void startLocalization() {
    // Reset AMCL state
    publishFakeHighCovariancePose();
    // Wait for the global localization service
    if (!client_->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Global localization service not available.");
      auto result = std::make_shared<Localize::Result>();
      result->success = false;
      result->status = "UNAVAILABLE";
      active_goal_->abort(result);
      return;
    }

    // Call global localization service
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Global localization triggered.");

    timer_ = this->create_wall_timer(
        100ms, std::bind(&GlobalLocalization::spinRobot, this));
  }

  void publishFakeHighCovariancePose() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    // Position and orientation can be zero
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    // Set very high covariance
    msg.pose.covariance[0] = 100.0;  // x
    msg.pose.covariance[7] = 100.0;  // y
    msg.pose.covariance[35] = 100.0; // yaw

    fake_pose_pub_->publish(msg);
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(),
                "Published fake high-covariance pose to /initialpose");
  }

  void spinRobot() {
    if (!active_goal_ || !rclcpp::ok())
      return;

    if (active_goal_->is_canceling()) {
      timer_->cancel();
      publishTwist(0.0);
      auto result = std::make_shared<Localize::Result>();
      result->success = false;
      result->status = "FAIL";
      active_goal_->canceled(result);
      return;
    }

    double direction = (counter_ < 250) ? 0.6 : -0.6;
    publishTwist(direction);
    counter_ = (counter_ + 1) % 500;

    if (cov_.empty() || !amcl_updated_)
      return;

    auto feedback = std::make_shared<Localize::Feedback>();
    feedback->cov_x = cov_[0];
    feedback->cov_y = cov_[7];
    feedback->cov_yaw = cov_[35];
    feedback->status = "PROGRESS";
    active_goal_->publish_feedback(feedback);
    float threshold = 0.15;
    if (env == "sim")
      threshold = 0.5;
    if (cov_[0] < threshold && cov_[7] < threshold && cov_[35] < threshold) {
      timer_->cancel();
      publishTwist(0.0);
      auto result = std::make_shared<Localize::Result>();
      result->success = true;
      result->status = "COMPLETE";
      active_goal_->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Localization complete.");
    }
  }

  void publishTwist(double z) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = z;
    cmd_vel_pub_->publish(twist);
    std::this_thread::sleep_for(200ms);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
