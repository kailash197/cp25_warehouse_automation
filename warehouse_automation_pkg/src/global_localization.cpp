#include "warehouse_automation_pkg/srv/global_localization.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using GlobalLocalizationSrv = warehouse_automation_pkg::srv::GlobalLocalization;

class GlobalLocalization : public rclcpp::Node {
public:
  GlobalLocalization() : Node("global_localization_service") {
    // Parameter: env
    std::string env = this->declare_parameter<std::string>("env", "real");
    std::string topic = (env == "sim")
                            ? "/diffbot_base_controller/cmd_vel_unstamped"
                            : "/cmd_vel";
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

    // Service client for AMCL
    amcl_gl_client_ = this->create_client<std_srvs::srv::Empty>(
        "/reinitialize_global_localization");

    // AMCL pose subscriber
    amcl_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          cov_ = msg->pose.covariance;
          amcl_updated = true;
          checkLocalization();
        });

    // Create global localization service
    gl_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/wh_global_localization_status", 10);
    gl_service_ = this->create_service<GlobalLocalizationSrv>(
        "wh_global_localization",
        [this](const std::shared_ptr<GlobalLocalizationSrv::Request> request,
               std::shared_ptr<GlobalLocalizationSrv::Response> response) {
          this->handleService(request, response);
        });
    RCLCPP_INFO(this->get_logger(), "Global Localization Service started.");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gl_status_pub_;

  void
  handleService(const std::shared_ptr<GlobalLocalizationSrv::Request> request,
                std::shared_ptr<GlobalLocalizationSrv::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received global localization request");
    publishStatus("READY");

    // Reset state
    cov_ = {};
    amcl_updated = false;
    counter_ = 0;
    localized_ = false;

    // Call AMCL service without spinning
    while (!amcl_gl_client_->wait_for_service(1s)) {
      response->success = false;
      response->message = "AMCL service not available";
      publishStatus("WAITING");
    }

    auto amcl_gl_req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto amcl_gl_future = amcl_gl_client_->async_send_request(amcl_gl_req);

    // Start spinning the robot
    timer_ = this->create_wall_timer(
        100ms, std::bind(&GlobalLocalization::spinRobot, this));

    response->success = true;
    response->message = "Localization Request Received.";
  }

  void spinRobot() {
    if (localized_)
      return;
    publishStatus("IN-PROGRESS");

    double direction = (counter_ < 250) ? 0.60 : -0.60;
    publishTwist(direction);
    counter_ = (counter_ + 1) % 500;
  }

  void checkLocalization() {
    if (cov_.empty() || !amcl_updated)
      return;

    if (cov_[0] < 0.07 && cov_[7] < 0.07 && cov_[35] < 0.07) {
      publishTwist(0.0);
      RCLCPP_INFO(this->get_logger(), "Localization successful.");
      timer_->cancel();
      localized_ = true;
      publishStatus("DONE");
    }
  }

  void publishStatus(std::string status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    gl_status_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Localization status: %s", status.c_str());
  }

  void publishTwist(double z) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = z;
    cmd_vel_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr amcl_gl_client_;
  rclcpp::Service<GlobalLocalizationSrv>::SharedPtr gl_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 36> cov_;
  int counter_ = 0;
  bool amcl_updated = false;
  bool localized_ = false;
  std::shared_ptr<GlobalLocalizationSrv::Response> current_response_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}