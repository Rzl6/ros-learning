#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp" // 包含你定义的接口

using base_interfaces_demo::action::Nav;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<Nav>;
using namespace std::placeholders;

// 3.定义节点类；
class NavActionClient : public rclcpp::Node {
public:
  NavActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("nav_client_node", node_options)
  {
    // 3-1.连接动作客户端；
    cli_ptr_ = rclcpp_action::create_client<Nav>(this, "nav_to_point");
  }

  // 3-2.发送请求；
  void send_goal(float tar_x, float tar_y)
  {

    if (!this->cli_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "动作客户端未被初始化。");
    }

    if (!this->cli_ptr_->wait_for_action_server(std::chrono::seconds(60))) {
      RCLCPP_ERROR(this->get_logger(), "服务连接失败！");
      return;
    }

    auto goal_msg = Nav::Goal();
    goal_msg.goal_x = tar_x;
    goal_msg.goal_y = tar_y;
    RCLCPP_INFO(this->get_logger(), "发送请求数据！");

    auto send_goal_options = rclcpp_action::Client<Nav>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&NavActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =std::bind(&NavActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&NavActionClient::result_callback, this, _1);
    auto goal_handle_future = this->cli_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Nav>::SharedPtr cli_ptr_;

  // 3-3.处理目标发送后的反馈；
  void goal_response_callback(GoalHandleNav::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝！");
    } else {
      RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中");
    }
  }

  // 3-4.处理连续反馈；
  void feedback_callback(GoalHandleNav::SharedPtr,const std::shared_ptr<const Nav::Feedback> feedback)
  {
    float remain_distance = feedback->remain_distance;
    RCLCPP_INFO(this->get_logger(), "当前距离目标: %.2f", remain_distance);
  }

  // 3-5.处理最终响应。
  void result_callback(const GoalHandleNav::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result && result.result->is_arrive) {
          RCLCPP_INFO(this->get_logger(), "任务成功：乌龟已到达！");
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "任务被中止");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "任务被取消");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知异常");
        return;
    }
  }
}; 

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // 1. 先检查用户有没有传参数进来
  // argc < 3 意味着用户只输了命令，没输坐标
  if (argc < 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "用法: ros2 launch 包名 launch文件名 x:=2, y:=2");
    return 1;
  }

  auto action_client = std::make_shared<NavActionClient>();

  // 2. 使用 atof 函数把字符串参数 (argv[]) 转浮点数 
  // 3. 把这个动态的数字传进去
  action_client->send_goal(atof(argv[1]), atof(argv[2]));

  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}