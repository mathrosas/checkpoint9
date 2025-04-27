#ifndef MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  /* helpers */
  bool publishCartFrame();

  /* callbacks */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void handleService(
      const std::shared_ptr<my_components::srv::GoToLoading::Request> request,
      std::shared_ptr<my_components::srv::GoToLoading::Response> response);
  void approachCart();

  /* parameters */
  std::vector<float> last_ranges_;
  std::vector<float> last_intensities_;
  std::vector<int> leg_indices_;
  int total_rays_{0};

  /* ROS interfaces */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Service<my_components::srv::GoToLoading>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
  rclcpp::TimerBase::SharedPtr approach_timer_;

  /* TF */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
