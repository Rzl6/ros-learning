#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("time_opt_demo");

  rclcpp::Time t1(1,500000000);
  rclcpp::Time t2(10,0);

  rclcpp::Duration du1(3,0);
  rclcpp::Duration du2(5,0);  

  //比较
  RCLCPP_INFO(node->get_logger(), "t1 >= t2 ? %d", (t1>=t2));
  RCLCPP_INFO(node->get_logger(), "t1 < t2 ? %d", t1<t2);
  //数学运算
  rclcpp::Time t3 = t2 + du1;
  rclcpp::Time t4 = t1 - du1;
  rclcpp::Duration du3 = t2 - t1;

  RCLCPP_INFO(node->get_logger(), "t3 = %.2f",t3.seconds());  
    RCLCPP_INFO(node->get_logger(), "t4 = %.2f",t4.seconds()); 
    RCLCPP_INFO(node->get_logger(), "du3 = %.2f",du3.seconds()); 

    RCLCPP_INFO(node->get_logger(),"--------------------------------------");
    // 比较
    RCLCPP_INFO(node->get_logger(),"du1 >= du2 ? %d", du1 >= du2);
    RCLCPP_INFO(node->get_logger(),"du1 < du2 ? %d", du1 < du2);
    // 数学运算
    rclcpp::Duration du4 = du1 * 3.0;
    rclcpp::Duration du5 = du1 + du2;
    rclcpp::Duration du6 = du1 - du2;

    RCLCPP_INFO(node->get_logger(), "du4 = %.2f",du4.seconds()); 
    RCLCPP_INFO(node->get_logger(), "du5 = %.2f",du5.seconds()); 
    RCLCPP_INFO(node->get_logger(), "du6 = %.2f",du6.seconds()); 

    rclcpp::shutdown();
    return 0;
}