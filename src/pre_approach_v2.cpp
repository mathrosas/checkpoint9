#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_v2") {
    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0);
    this->declare_parameter("final_approach", false);

    getting_params();

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PreApproach::laser_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diffbot_base_controller/odom", 10,
        std::bind(&PreApproach::odom_callback, this, _1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer_callback, this));

    client_ =
        this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    while (!client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for service /approach_shelf to become available...");
    }

    RCLCPP_INFO(this->get_logger(), "Starting forward motion...");
  }

private:
  void getting_params() {
    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();
    final_approach_ = this->get_parameter("final_approach").as_bool();
  }

  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) const {
    float desired_angle_rad = desired_angle_deg * M_PI / 180.0;
    int index = static_cast<int>(std::round(
        (desired_angle_rad - scan->angle_min) / scan->angle_increment));
    return (index + scan->ranges.size()) % scan->ranges.size();
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int index = angle_to_index(msg, 0);
    front_laser_value_ = msg->ranges[index];
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto orientation = msg->pose.pose.orientation;
    double siny_cosp =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y +
                                    orientation.z * orientation.z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    is_odom_received_ = true;
  }

  void timer_callback() {
    geometry_msgs::msg::Twist cmd;

    if (!stopped_) {
      if (front_laser_value_ < obstacle_) {
        RCLCPP_INFO(this->get_logger(), "Obstacle at %.2f m. Stopping...",
                    front_laser_value_);
        cmd.linear.x = 0.0;
        stopped_ = true;
        waiting_to_start_rotation_ = true;
        stop_time_ = this->now();
      } else {
        cmd.linear.x = 0.2;
      }
      vel_pub_->publish(cmd);
    } else if (waiting_to_start_rotation_) {
      if ((this->now() - stop_time_).seconds() > 0.5 && is_odom_received_) {
        start_yaw_ = current_yaw_;
        target_yaw_ = normalize_angle(start_yaw_ + degrees_ * M_PI / 180.0);
        waiting_to_start_rotation_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Captured start_yaw=%.2f, target_yaw=%.2f", start_yaw_,
                    target_yaw_);
      }
    } else if (!rotated_) {
      double angle_diff = normalize_angle(target_yaw_ - current_yaw_);
      RCLCPP_INFO(this->get_logger(), "Angle difference: %.4f rad", angle_diff);

      if (std::abs(angle_diff) < 0.15) {
        cmd.angular.z = 0.0;
        rotated_ = true;
        vel_pub_->publish(cmd);

        // Now call the service
        auto request =
            std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach_;

        client_->async_send_request(request,
                                    std::bind(&PreApproach::handle_response,
                                              this, std::placeholders::_1));
      } else {
        cmd.angular.z = (angle_diff > 0) ? 0.5 : -0.5;
        vel_pub_->publish(cmd);
      }
    }
  }

  void handle_response(
      const rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture
          future) {
    auto response = future.get();
    if (response->complete) {
      RCLCPP_INFO(this->get_logger(),
                  "Two legs detected and final approach performed.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Final approach not performed.");
    }
    rclcpp::shutdown();
  }

  // Parameters
  double obstacle_;
  int degrees_;
  bool final_approach_;
  double front_laser_value_ = 10.0;
  double current_yaw_ = 0.0;
  double start_yaw_ = 0.0;
  double target_yaw_ = 0.0;

  // State
  bool is_odom_received_ = false;
  bool stopped_ = false;
  bool rotated_ = false;
  bool waiting_to_start_rotation_ = false;
  rclcpp::Time stop_time_;

  // ROS Interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
