#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "base_interfaces_demo/action/nav.hpp" // 包含你定义的接口

using base_interfaces_demo::action::Nav;
using GoalHandleNav = rclcpp_action::ServerGoalHandle<Nav>;

class NavActionServer : public rclcpp::Node {
public:
    NavActionServer() : Node("nav_server_node") {
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
    float cur_x, cur_y; // 当前坐标

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        cur_x = msg->x;
        cur_y = msg->y;
    }

    // 核心执行逻辑（在后台线程运行）
    void execute(const std::shared_ptr<GoalHandleNav> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Nav::Feedback>();
        auto result = std::make_shared<Nav::Result>();
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        rclcpp::Rate loop_rate(10); // 10Hz 循环

        while (rclcpp::ok()) {
            // --- 你的逻辑挑战区 ---
            
            // 1. 计算当前距离目标的距离
            float distance = sqrt(pow(goal->goal_x - cur_x, 2) + pow(goal->goal_y - cur_y, 2));

            // 2. 检查是否被取消任务
            if (goal_handle->is_canceling()) {
                result->is_arrive = false;
                goal_handle->canceled(result);
                return;
            }

            // 3. 组织并发布反馈数据 (remain_distance)
            // TODO: 填充 feedback 并调用发布函数

            // 4. 判断是否到达目标（比如距离小于 0.1 米）
            if (distance < 0.1) {
                // TODO: 停止乌龟运动（发一个速度为 0 的消息），设置结果并返回成功
                break;
            }

            // 5. 简单的运动逻辑：给个前进速度
            cmd_vel.linear.x = 1.0; // 这里简单化，只管往前冲
            pub_ptr_->publish(cmd_vel);

            loop_rate.sleep();
        }
    }

    // 处理接受后的逻辑（开线程）
    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle) {
        std::thread{std::bind(&NavActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // 省略 handle_goal 和 handle_cancel ...
};