#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // 乌龟 1 的位姿（包含速度）与发给乌龟 2 的速度指令

//乌龟1初始旋转180度
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/action/rotate_absolute.hpp"

using RotateAbs = turtlesim::action::RotateAbsolute;
using GoalHandleRotate = rclcpp_action::ClientGoalHandle<RotateAbs>;

class MirrorControl : public rclcpp::Node {
public:
    MirrorControl() : Node("ex01_mirror_control") {
    
        // 1. 创建发布者：发布乌龟 2 的速度指令 "/t2/turtle1/cmd_vel"
        pub_ptr_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/t2/turtle1/cmd_vel",
            10
        );
        // TODO: 补充发布者代码

        // 2. action发送请求，并开始转弯，乌龟2先转弯
        action_client_ = rclcpp_action::create_client<RotateAbs>(this, "/t2/turtle1/rotate_absolute");
        
        this->send_rotate_goal();
    }

private:
    //初始接收action,让乌龟2转180度
    void send_rotate_goal(){
        //等待服务端上线
        if(!this->action_client_->wait_for_action_server(std::chrono::seconds(10))){
            RCLCPP_ERROR(this->get_logger(), "服务链接失败");
            return; 
        }

        auto goal_msg = RotateAbs::Goal();
        goal_msg.theta = M_PI;

        RCLCPP_INFO(this->get_logger(), "发送初始旋转角度中");
        
        auto send_goal_options = rclcpp_action::Client<RotateAbs>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&MirrorControl::result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);

    }    

    void result_callback(const GoalHandleRotate::WrappedResult & result){
        if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
            
            RCLCPP_INFO(this->get_logger(), "乌龟2已转向, 开启订阅");
            
            // 4. 创建订阅者：订阅乌龟 1 的位姿话题 "/t1/turtle1/cmd_vel"
            sub_ptr_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/t1/turtle1/cmd_vel",
                10,
                std::bind(&MirrorControl::sub_callback, this, std::placeholders::_1)
            );
            // TODO: 补充订阅者代码，回调函数设为 sub_callback  
        }
              
    }

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

    //声明action服务端
    rclcpp_action::Client<RotateAbs>::SharedPtr action_client_;
    // TODO: 补全声明
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MirrorControl>());
    rclcpp::shutdown();
    return 0;
}