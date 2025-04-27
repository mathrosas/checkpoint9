#include <memory>

#include "my_components/attach_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto attach_server = std::make_shared<my_components::AttachServer>(options);
  exec.add_node(attach_server);

  RCLCPP_INFO(attach_server->get_logger(),
              "Manual composition started; waiting for /approach_shelf calls.");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
