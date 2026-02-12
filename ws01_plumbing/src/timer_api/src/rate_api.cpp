#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rate_demo");
  rclcpp::Rate rate(1.0);
  int32_t cnt=0;
  while(rclcpp::ok())
  {
    RCLCPP_INFO(node->get_logger(), "hello rate %d", cnt++);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}