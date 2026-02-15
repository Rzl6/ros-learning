#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp" // 包含你定义的接口


using base_interfaces_demo::action::Nav;
using GoalHandleNav = rclcpp_action::ServerGoalHandle<Nav>;

class NavActionServer : public rclcpp::Node {
public:
    NavActionServer() : Node("nav_server_node"), cur_x(0.0), cur_y(0.0), cur_theta(0.0) {
        // 1. 初始化发布者：发布速度控制小乌龟
        pub_ptr_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // 2. 初始化订阅者：获取乌龟当前位置
        sub_ptr_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&NavActionServer::pose_callback, this, std::placeholders::_1));

        // 3. 初始化 Action 服务端
        server_ptr_ = rclcpp_action::create_server<Nav>(
            this, "nav_to_point",
            std::bind(&NavActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&NavActionServer::handle_accepted, this, std::placeholders::_1)
        );

    }

private:
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ptr_;    
    rclcpp_action::Server<Nav>::SharedPtr server_ptr_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_ptr_;
    float cur_x, cur_y, cur_theta; // 当前坐标与角度

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        cur_theta = msg->theta;
        cur_x = msg->x;
        cur_y = msg->y;
    }

    // 核心执行逻辑（在后台线程运行）
    void execute(const std::shared_ptr<GoalHandleNav> goal_handle) {
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Nav::Feedback>();
        auto result = std::make_shared<Nav::Result>();
        auto cmd_vel = geometry_msgs::msg::Twist();

        while (rclcpp::ok()) {
            // 1. 实时计算距离目标点的距离（剩下的路程）
            float distance = sqrt(pow(goal->goal_x - cur_x, 2) + pow(goal->goal_y - cur_y, 2));

            // 2. 实时计算目标角度（我现在应该朝哪看？）
            float target_theta = atan2(goal->goal_y - cur_y, goal->goal_x - cur_x);

            // 3. 实时计算角度偏差（我偏离了多少？）
            float theta_error = target_theta - cur_theta;

            // 【重点】处理角度旋转方向的优化（防止乌龟为了转10度而绕一圈）
            // 如果误差大于180度，就反向转
            if (theta_error > M_PI) theta_error -= 2 * M_PI;
            if (theta_error < -M_PI) theta_error += 2 * M_PI;

            // 4. 检查是否被取消
            if (goal_handle->is_canceling()) {
                result->is_arrive = false;
                goal_handle->canceled(result);
                return;
            }

            // 5. 判断是否到达（停止条件）
            if (distance < 0.01) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                pub_ptr_->publish(cmd_vel);
                break;
            }

            // 6. 【核心算法：比例控制】
            // 距离越远，跑得越快；角度偏得越多，转得越猛
            cmd_vel.linear.x = 0.5 * distance;   // 线速度比例系数 0.5
            cmd_vel.angular.z = 2.0 * theta_error; // 角速度比例系数 2.0

            // 限制最大速度，防止乌龟飞出屏幕
            if (cmd_vel.linear.x > 2.0) cmd_vel.linear.x = 2.0;

            // 7. 发布反馈和速度
            feedback->remain_distance = distance;
            goal_handle->publish_feedback(feedback);
            pub_ptr_->publish(cmd_vel);

            loop_rate.sleep();
        }

        // 循环结束，任务成功
        if (rclcpp::ok()) {
            result->is_arrive = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "任务完成！乌龟已归位。");
        }
    }

    // 处理接受后的逻辑（开线程）
    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle) {
        std::thread{std::bind(&NavActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Nav::Goal> goal){
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "接收到动作客户端请求，请求坐标为(%.2f, %.2f)", goal->goal_x, goal->goal_y);
        if (goal->goal_x<0 && goal->goal_y<0) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "接收到任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
};

int main(int argc, char ** argv)
{
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针；
  auto action_server = std::make_shared<NavActionServer>();
  rclcpp::spin(action_server);
  // 5.释放资源。
  rclcpp::shutdown();
  return 0;
}