#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // 乌龟 1 的位姿（包含速度）与发给乌龟 2 的速度指令

//ros2 topic info /t2/turtle1/cmd_vel
//ros2 interface show geometry_msgs/msg/Twist


class MirrorControl : public rclcpp::Node {
public:
    MirrorControl() : Node("ex01_mirror_control") {
        // 1. 创建订阅者：订阅乌龟 1 的位姿话题 "/t1/turtle1/cmd_vel"
        sub_ptr_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/t1/turtle1/cmd_vel",
            10,
            std::bind(&MirrorControl::sub_callback, this, std::placeholders::_1)
        );
        // TODO: 补充订阅者代码，回调函数设为 sub_callback
        
        // 2. 创建发布者：发布乌龟 2 的速度指令 "/t2/turtle1/cmd_vel"
        pub_ptr_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/t2/turtle1/cmd_vel",
            10
        );
        // TODO: 补充发布者代码
    }

private:
    // 回调函数：当收到乌龟 1 的位姿时触发
    void sub_callback(geometry_msgs::msg::Twist::SharedPtr pose) {
        RCLCPP_INFO(this->get_logger(), "乌龟1线速度: %.2f, 角速度: %.2f", 
            pose->linear.x, pose->angular.z);
        // 3. 组织发给乌龟 2 的消息
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = pose->linear.x;
        twist.angular.z = -pose->angular.z;
        // TODO: 根据你刚才说的镜像逻辑，给 twist.linear.x 和 twist.angular.z 赋值
        // 提示：pose->linear_velocity 是乌龟 1 当前的线速度
        // 提示：pose->angular_velocity 是乌龟 1 当前的角速度
        
        // 4. 发布出去
        pub_ptr_->publish(twist);
        // TODO: 执行发布动作
    }

    // 声明订阅者和发布者指针
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ptr_;

    // TODO: 补全声明
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MirrorControl>());
    rclcpp::shutdown();
    return 0;
}