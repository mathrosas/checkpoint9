#include "my_components/attach_server.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace my_components {

/* constructor */
AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("approach_service_server", options) {
  /* TF */
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  /* interfaces */
  srv_ = create_service<my_components::srv::GoToLoading>(
      "/approach_shelf", std::bind(&AttachServer::handleService, this, _1, _2));

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&AttachServer::laserCallback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 1);
  elevator_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 1);

  RCLCPP_INFO(get_logger(), "Approach shelf service (component) ready.");
}

/* laser callback */

void AttachServer::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

/* service handler */

void AttachServer::handleService(
    const std::shared_ptr<my_components::srv::GoToLoading::Request> request,
    std::shared_ptr<my_components::srv::GoToLoading::Response> response) {
  if (leg_indices_.size() != 2) {
    RCLCPP_WARN(get_logger(), "Need exactly two reflective blobs, found %zu.",
                leg_indices_.size());
    response->complete = false;
    return;
  }

  if (!publishCartFrame()) {
    response->complete = false;
    return;
  }

  if (!request->attach_to_shelf) {
    response->complete = false;
    return;
  }

  response->complete = true;

  /* start the periodic motion controller */
  approach_timer_ =
      create_wall_timer(100ms, std::bind(&AttachServer::approachCart, this));
}

/* helper: cart TF */

bool AttachServer::publishCartFrame() {
  const int idx1 = leg_indices_[0];
  const int idx2 = leg_indices_[1];

  const float angle_min = -M_PI;
  const float angle_increment = (2.0F * M_PI) / last_ranges_.size();
  const float angle1 = angle_min + idx1 * angle_increment;
  const float angle2 = angle_min + idx2 * angle_increment;

  const float r1 = last_ranges_[idx1];
  const float r2 = last_ranges_[idx2];

  const float mid_x = (r1 * std::cos(angle1) + r2 * std::cos(angle2)) / 2.0F;
  const float mid_y = (r1 * std::sin(angle1) + r2 * std::sin(angle2)) / 2.0F;

  geometry_msgs::msg::PointStamped laser_pt, odom_pt;
  laser_pt.header.frame_id = "robot_front_laser_base_link";
  laser_pt.header.stamp = now();
  laser_pt.point.x = mid_x;
  laser_pt.point.y = mid_y;

  try {
    const auto tf = tf_buffer_->lookupTransform(
        "odom", "robot_front_laser_base_link", tf2::TimePointZero,
        tf2::durationFromSec(0.5));
    tf2::doTransform(laser_pt, odom_pt, tf);

    geometry_msgs::msg::TransformStamped cart_tf;
    cart_tf.header.stamp = now();
    cart_tf.header.frame_id = "odom";
    cart_tf.child_frame_id = "cart_frame";
    cart_tf.transform.translation.x = odom_pt.point.x;
    cart_tf.transform.translation.y = odom_pt.point.y;
    cart_tf.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(cart_tf);
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "TF error while publishing cart_frame: %s",
                 ex.what());
    return false;
  }
}

/* periodic approach loop */

void AttachServer::approachCart() {
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform("robot_base_link", "cart_frame",
                                    tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }

  const double x = t.transform.translation.x + 0.5;
  const double y = t.transform.translation.y;
  const double dist = std::hypot(x, y);
  const double yaw_err = std::atan2(y, x);

  geometry_msgs::msg::Twist cmd;
  if (dist > 0.03) {
    cmd.linear.x = std::clamp(dist, 0.0, 0.3);
    cmd.angular.z = -0.5 * yaw_err;
  } else {
    cmd.linear.x = cmd.angular.z = 0.0;
    approach_timer_->cancel();

    RCLCPP_INFO(
        get_logger(),
        "Final approach complete. Publishing /elevator_up String message.");

    std_msgs::msg::String up_msg;
    up_msg.data = "up";
    elevator_pub_->publish(up_msg);
  }
  vel_pub_->publish(cmd);
}

} // namespace my_components

/* register with rclcpp_components */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)