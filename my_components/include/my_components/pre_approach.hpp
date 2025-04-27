#ifndef MY_COMPONENTS__PRE_APPROACH_HPP_
#define MY_COMPONENTS__PRE_APPROACH_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

private:
  /* helpers */
  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) const;
  double normalize_angle(double angle);

  /* callbacks */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();

  /* parameters */
  double obstacle_{0.3};
  int degrees_{-90};

  /* state */
  double front_laser_value_{10.0};
  double current_yaw_{0.0};
  double start_yaw_{0.0};
  double target_yaw_{0.0};

  bool is_odom_received_{false};
  bool stopped_{false};
  bool rotated_{false};
  bool waiting_to_start_rotation_{false};
  rclcpp::Time stop_time_;

  /* ROS interfaces */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // MY_COMPONENTS__PRE_APPROACH_HPP_
