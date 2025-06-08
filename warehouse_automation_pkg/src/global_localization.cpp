#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class GlobalLocalization : public rclcpp::Node {
public:
  GlobalLocalization() : Node("global_localization") {
    // Parameter: env
    std::string env = this->declare_parameter<std::string>("env", "real");

    // Publisher based on env
    std::string topic = (env == "sim")
                            ? "/diffbot_base_controller/cmd_vel_unstamped"
                            : "/cmd_vel";
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

    // Service client
    client_ = this->create_client<std_srvs::srv::Empty>(
        "/reinitialize_global_localization");

    // AMCL pose subscriber
    amcl_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          cov_ = msg->pose.covariance;
          amcl_updated = true;
        });

    // Start spinning after short delay
    timer_ = this->create_wall_timer(
        100ms, std::bind(&GlobalLocalization::spinRobot, this));

    RCLCPP_INFO(this->get_logger(), "GlobalLocalization node started.");
  }

  void wait_and_call_service() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok())
        return;
    }

    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = client_->async_send_request(req);
    rclcpp::spin_until_future_complete(shared_from_this(), future);
    RCLCPP_INFO(this->get_logger(), "Called global localization service.");
  }

private:
  void spinRobot() {
    double direction = (counter_ < 250) ? 0.35 : -0.35;
    publishTwist(direction);
    counter_ = (counter_ + 1) % 500;

    if (cov_.empty())
      return;

    if (!amcl_updated)
      return;
    if (cov_[35] < 0.1) {
      RCLCPP_INFO(this->get_logger(), "cov[35] (yaw variance): %.6f", cov_[35]);
    }

    if (cov_[0] < 0.07 && cov_[7] < 0.07 && cov_[35] < 0.07) {
      publishTwist(0.0);
      RCLCPP_INFO(this->get_logger(), "Localized. Shutting down.");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }
  }

  void publishTwist(double z) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = z;
    cmd_vel_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<double, 36> cov_;
  int counter_ = 0;
  bool amcl_updated = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalization>();
  node->wait_and_call_service();
  rclcpp::spin(node); // Spin will return when timer is cancelled
  rclcpp::shutdown(); // Safe shutdown after spin ends
  return 0;
}
