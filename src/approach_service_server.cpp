#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace std::placeholders;
using namespace std::chrono_literals;

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("approach_service_server") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    srv_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachService::handle_service, this, _1, _2));

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ApproachService::laser_callback, this, _1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    elevator_pub_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    RCLCPP_INFO(this->get_logger(), "Approach shelf service is ready.");
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_ranges_ = msg->ranges;
    last_intensities_ = msg->intensities;
    total_rays_ = static_cast<int>(msg->ranges.size());

    leg_indices_.clear();
    bool in_blob = false;
    int blob_start = 0;
    const double INTENSITY_THRESH = 1000.0;

    for (int i = 0; i < static_cast<int>(last_intensities_.size()); ++i) {
      if (last_intensities_[i] > INTENSITY_THRESH) {
        if (!in_blob) {
          in_blob = true;
          blob_start = i;
        }
      } else if (in_blob) {
        in_blob = false;
        leg_indices_.push_back((blob_start + i) / 2);
      }
    }
  }

  void handle_service(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    if (leg_indices_.size() != 2) {
      RCLCPP_WARN(this->get_logger(),
                  "Need exactly two reflective blobs, found %zu.",
                  leg_indices_.size());
      response->complete = false;
      return;
    }

    if (!publish_cart_frame()) {
      response->complete = false;
      return;
    }

    if (!request->attach_to_shelf) {
      response->complete = false;
      return;
    }

    response->complete = true;
    approach_timer_ = this->create_wall_timer(
        100ms, std::bind(&ApproachService::approach_cart, this));
  }

  bool publish_cart_frame() {
    int idx1 = leg_indices_[0];
    int idx2 = leg_indices_[1];

    float angle_min = -M_PI;
    float angle_increment = (2 * M_PI) / last_ranges_.size();
    float angle1 = angle_min + idx1 * angle_increment;
    float angle2 = angle_min + idx2 * angle_increment;

    float r1 = last_ranges_[idx1];
    float r2 = last_ranges_[idx2];

    float mid_x = (r1 * std::cos(angle1) + r2 * std::cos(angle2)) / 2.0f;
    float mid_y = (r1 * std::sin(angle1) + r2 * std::sin(angle2)) / 2.0f;

    geometry_msgs::msg::PointStamped laser_pt, odom_pt;
    laser_pt.header.frame_id = "robot_front_laser_base_link";
    laser_pt.header.stamp = this->now();
    laser_pt.point.x = mid_x;
    laser_pt.point.y = mid_y;

    try {
      auto tf = tf_buffer_->lookupTransform(
          "odom", "robot_front_laser_base_link", tf2::TimePointZero,
          tf2::durationFromSec(0.5));
      tf2::doTransform(laser_pt, odom_pt, tf);

      geometry_msgs::msg::TransformStamped cart_tf;
      cart_tf.header.stamp = this->now();
      cart_tf.header.frame_id = "odom";
      cart_tf.child_frame_id = "cart_frame";
      cart_tf.transform.translation.x = odom_pt.point.x;
      cart_tf.transform.translation.y = odom_pt.point.y;
      cart_tf.transform.rotation.w = 1.0;

      tf_static_broadcaster_->sendTransform(cart_tf);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "TF error while publishing cart_frame: %s", ex.what());
      return false;
    }
  }

  void approach_cart() {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("robot_base_link", "cart_frame",
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      return;
    }

    double x = t.transform.translation.x + 0.5;
    double y = t.transform.translation.y;

    double dist = std::hypot(x, y);
    double yaw_err = std::atan2(y, x);

    geometry_msgs::msg::Twist cmd;
    if (dist > 0.015) {
      cmd.linear.x = std::clamp(dist, 0.0, 0.3);
      cmd.angular.z = -0.6 * yaw_err;
    } else {
      cmd.linear.x = cmd.angular.z = 0.0;
      approach_timer_->cancel();
      RCLCPP_INFO(
          this->get_logger(),
          "Final approach complete. Publishing /elevator_up String message.");

      std_msgs::msg::String up_msg;
      up_msg.data = "up";
      elevator_pub_->publish(up_msg);
    }
    vel_pub_->publish(cmd);
  }

  std::vector<float> last_ranges_;
  std::vector<float> last_intensities_;
  int total_rays_{0};
  std::vector<int> leg_indices_;

  rclcpp::TimerBase::SharedPtr approach_timer_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachService>());
  rclcpp::shutdown();
  return 0;
}
