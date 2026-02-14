#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp" // 你刚才定义的接口
#include <cmath> // 勾股定理开根号要用 sqrt()

using base_interfaces_demo::srv::Distance;
using std::placeholders::_1;
using std::placeholders::_2;

class DistanceServer : public rclcpp::Node {
public:
    DistanceServer() : Node("distance_server_node"), x1(0.0), y1(0.0){
        // 1. 【初始化订阅者】监听乌龟1的位置
        sub_ptr_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&DistanceServer::pose_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "已经监听到乌龟");
        // TODO: 订阅话题 "/turtle1/pose", 回调函数 pose_callback
        
        // 2. 【初始化服务端】提供计算服务
        svr_ptr_ = this->create_service<Distance>(
            "get_distance", 
            std::bind(&DistanceServer::handle_distance, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "服务端已创建");
        // TODO: 创建服务 "get_distance", 回调函数 handle_distance
    }

private:
    // 成员变量：存储乌龟 1 的最新位置
    float x1, y1;

    // 话题回调函数：不断刷新乌龟 1 的坐标
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        x1 = msg->x;
        y1 = msg->y;
    }

    // 服务回调函数：收到乌龟 2 坐标，计算距离
    void handle_distance(const std::shared_ptr<Distance::Request> request,
                         std::shared_ptr<Distance::Response> response) 
    {
        // 3. 【数学逻辑实现】
        // 乌龟 1 坐标：(x1, y1)
        // 乌龟 2 坐标：(request->x, request->y)
        float_t x = x1 - request->x;
        float_t y = y1 - request->y;
        response->distance = sqrt(x*x + y*y);
        // TODO: 计算两点距离并赋值给 response->distance
        // 提示：公式是 sqrt( (x2-x1)^2 + (y2-y1)^2 )
        
        RCLCPP_INFO(this->get_logger(), "计算请求：x1=%.2f, y1=%.2f | x2=%.2f, y2=%.2f", x1, y1, request->x, request->y);
        RCLCPP_INFO(this->get_logger(), "计算结果：%.2f", response->distance);
    }

    // 声明组件指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_ptr_;
    rclcpp::Service<Distance>::SharedPtr svr_ptr_;


    // TODO: 补全声明
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<DistanceServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}