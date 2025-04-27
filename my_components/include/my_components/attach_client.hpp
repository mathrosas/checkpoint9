#ifndef MY_COMPONENTS__ATTACH_CLIENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_HPP_

#include "my_components/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  /* timer trigger ------------------------------------------------------ */
  void on_timer();

  /* parameters --------------------------------------------------------- */
  bool final_approach_{true}; // send true → server starts motion

  /* ROS interface ------------------------------------------------------ */
  rclcpp::Client<my_components::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_CLIENT_HPP_
