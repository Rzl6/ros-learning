#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp" // 你刚才定义的接口


using base_interfaces_demo::srv::Distance;
using std::placeholders::_1;
using std::placeholders::_2;

//定义节点类
class DistanceServer : public rclcpp::Node {
public:
    DistanceServer() : Node("distance_server_node"), x1(0.0), y1(0.0){
        //创造客户端
        client_ = this->create_client<Distance>("add_ints");
        RCLCPP_INFO(this->get_logger(), "客户端创建，等待连接服务器...");
    }
    
        //等待服务连接
        bool connect_server()
        {
            while(!client_->wait_for_service(1s))
            {
                if(!rclcpp::ok()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强制退出！");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "服务连接中, 请稍候...");
            }
            return true;
        }
        rclcpp::Client<AddInts>::FutureAndRequestId send_request(int32_t num1, int32_t num2)
        {
            auto request_ = std::make_shared<AddInts::Request>();
            request_->num1 = num1;
            request_->num2 = num2;
            return client_->async_send_request(request_);
        }

    private:
        rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    if(argc != 3){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请提交两个整型数据");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto client = std::make_shared<MinimalClient>();
    bool flag = client->connect_server();
    if(!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接失败！");

    }
    auto response = client->send_request(atoi(argv[1]), atoi(argv[2]));

    //处理响应
    if(rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(), "请求正常处理");
        RCLCPP_INFO(client->get_logger(), "响应结果：%d", response.get()->sum);

    }else{
        RCLCPP_INFO(client->get_logger(), "请求异常");
    }

    rclcpp::shutdown();
    return 0;

}
