#include "rclcpp/rclcpp.hpp"

int main(int argc, char const **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("time_demo");

  rclcpp::Time t1(10500000000L);
  rclcpp::Time t2(2,1000000000L);
  rclcpp::Time roght_now = node->now();
  
  RCLCPP_INFO(node->get_logger(),"s = %.2f, ns = %ld",t1.seconds(),t1.nanoseconds());
  RCLCPP_INFO(node->get_logger(),"s = %.2f, ns = %ld",t2.seconds(),t2.nanoseconds());
  RCLCPP_INFO(node->get_logger(),"s = %.2f, ns = %ld",roght_now.seconds(),roght_now.nanoseconds());

  rclcpp::shutdown();
  return 0;
}