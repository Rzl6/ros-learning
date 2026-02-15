#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "base_interfaces_demo/srv/distance.hpp" // 你刚才定义的接口


using base_interfaces_demo::srv::Distance;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

//定义节点类
class DistanceClient : public rclcpp::Node {
public:
    DistanceClient() : Node("distance_server_node"){
        //创造客户端
        spw_cli_ptr_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        dis_cli_ptr_ = this->create_client<Distance>("get_distance");
        RCLCPP_INFO(this->get_logger(), "客户端创建，等待连接服务器...");
    }
    
    //等待服务连接
    bool spw_connect_server(){
        while(!spw_cli_ptr_->wait_for_service(1s))
        {
            if(!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强制退出！");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "spawn服务连接中, 请稍候...");
        }
        return true;
    }
    bool dis_connect_server(){
        while(!dis_cli_ptr_->wait_for_service(1s))
        {
            if(!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强制退出！");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "distance服务连接中, 请稍候...");
        }
        return true;
    }
    
    //发送请求
    rclcpp::Client<turtlesim::srv::Spawn>::FutureAndRequestId spw_send_request(float tar_x, float tar_y){
        auto request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
        request_->x = tar_x;
        request_->y = tar_y;
        return spw_cli_ptr_->async_send_request(request_);
    }
    rclcpp::Client<Distance>::FutureAndRequestId dis_send_request(float tar_x, float tar_y){
        auto request_ = std::make_shared<Distance::Request>();
        request_->x = tar_x;
        request_->y = tar_y;
        return dis_cli_ptr_->async_send_request(request_);
    }

    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spw_cli_ptr_;    
        rclcpp::Client<Distance>::SharedPtr dis_cli_ptr_;
};

int main(int argc, char **argv)
{
    if(argc < 3){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "用法: ros2 launch ... x:=数字 y:=数字");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto client = std::make_shared<DistanceClient>();
    bool spw_flag = client->spw_connect_server();
    if(!spw_flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败！");
    }
    auto response = client->spw_send_request(atof(argv[1]), atof(argv[2]));

    //处理响应
    if(rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(), "/spawn请求正常处理, 开始连接distance客户端");
        bool dis_flag = client->dis_connect_server();
        if(!dis_flag){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败！");
        }
        auto dis_response = client->dis_send_request(atof(argv[1]), atof(argv[2]));
        if(rclcpp::spin_until_future_complete(client,dis_response) == rclcpp::FutureReturnCode::SUCCESS){
            auto result = dis_response.get();
            RCLCPP_INFO(client->get_logger(), "两龟距离为：%.2f", result->distance);    
        }else{
            RCLCPP_INFO(client->get_logger(), "请求异常");
        }
    }else{
        RCLCPP_INFO(client->get_logger(), "请求异常");
    }

    rclcpp::shutdown();
    return 0;

}
