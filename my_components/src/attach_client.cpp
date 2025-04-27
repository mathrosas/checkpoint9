#include "my_components/attach_client.hpp"

using namespace std::chrono_literals;
using ServiceFuture =
    rclcpp::Client<my_components::srv::GoToLoading>::SharedFuture;

namespace my_components {

/* constructor */

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {
  client_ = create_client<my_components::srv::GoToLoading>("/approach_shelf");

  timer_ = create_wall_timer(500ms, std::bind(&AttachClient::on_timer, this));
}

/* timer callback */

void AttachClient::on_timer() {
  /* run just once */
  timer_->cancel();

  if (!client_->wait_for_service(1s)) {
    RCLCPP_WARN(get_logger(),
                "Service /approach_shelf not available yet – giving up.");
    return;
  }

  /* build request */
  auto request = std::make_shared<my_components::srv::GoToLoading::Request>();
  request->attach_to_shelf = final_approach_;

  /* async call with inline response handler */
  auto response = [this](ServiceFuture fut) {
    const auto resp = fut.get();
    if (resp->complete) {
      RCLCPP_INFO(get_logger(),
                  "Server replied: final approach completed successfully.");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Server replied: final approach NOT performed.");
    }
    /* Nothing else to do; node now sits idle and the server keeps running. */
  };

  client_->async_send_request(request, response);
}

} // namespace my_components

/* register with rclcpp_components */
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
