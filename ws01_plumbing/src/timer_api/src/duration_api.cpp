#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("duration_node");

  rclcpp::Duration du1(1s);
  rclcpp::Duration du2(2, 500000000);

  RCLCPP_INFO(node->get_logger(), "s = %.2f, ns = %ld", du1.seconds(), du2.nanoseconds());

  rclcpp::shutdown();
  return 0;

}