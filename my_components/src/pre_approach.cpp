#include "my_components/pre_approach.hpp"

#include <cmath>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach", options) {

  /* subscriptions                                                 */
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&PreApproach::laser_callback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diffbot_base_controller/odom", 10,
      std::bind(&PreApproach::odom_callback, this, _1));

  /* publisher                                                     */
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  /* timer                                                         */
  timer_ =
      create_wall_timer(100ms, std::bind(&PreApproach::timer_callback, this));
}

/* helpers */
int PreApproach::angle_to_index(
    const sensor_msgs::msg::LaserScan::SharedPtr &scan,
    float desired_angle_deg) const {
  const float desired_angle_rad = desired_angle_deg * M_PI / 180.0;
  int index = static_cast<int>(std::round(
      (desired_angle_rad - scan->angle_min) / scan->angle_increment));
  return (index + scan->ranges.size()) % scan->ranges.size();
}

double PreApproach::normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/* ---------- callbacks ---------- */
void PreApproach::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  const int idx = angle_to_index(msg, 0);
  front_laser_value_ = msg->ranges[idx];
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto &q = msg->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  is_odom_received_ = true;
}

void PreApproach::timer_callback() {
  geometry_msgs::msg::Twist cmd;

  if (!stopped_) {
    if (front_laser_value_ < obstacle_) {
      RCLCPP_INFO(get_logger(), "Obstacle at %.2f m → stopping",
                  front_laser_value_);
      cmd.linear.x = 0.0;
      vel_pub_->publish(cmd);

      stopped_ = true;
      waiting_to_start_rotation_ = true;
      stop_time_ = now();
    } else {
      cmd.linear.x = 0.2;
      vel_pub_->publish(cmd);
    }
    return;
  }

  if (waiting_to_start_rotation_) {
    if ((now() - stop_time_).seconds() > 0.5 && is_odom_received_) {
      start_yaw_ = current_yaw_;
      target_yaw_ = normalize_angle(start_yaw_ + degrees_ * M_PI / 180.0);
      waiting_to_start_rotation_ = false;

      RCLCPP_INFO(get_logger(), "Captured start_yaw=%.2f, target_yaw=%.2f",
                  start_yaw_, target_yaw_);
    }
    return;
  }

  if (!rotated_) {
    const double angle_diff = normalize_angle(target_yaw_ - current_yaw_);
    RCLCPP_INFO(get_logger(), "Angle diff: %.4f rad", angle_diff);

    if (std::fabs(angle_diff) < 0.15) {
      cmd.angular.z = 0.0;
      rotated_ = true;
      RCLCPP_INFO(get_logger(), "Finished rotating %d deg", degrees_);
    } else {
      cmd.angular.z = (angle_diff > 0) ? 0.5 : -0.5;
    }
    vel_pub_->publish(cmd);
  }
}

/* register with pluginlib */
} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
