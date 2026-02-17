#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class ColorChanger : public rclcpp::Node {
public:
    // 构造函数
    ColorChanger() : Node("color_changer_node"), r_(0), g_(0), b_(0) {
        // 1. 【改动点】换成异步客户端 (AsyncParametersClient)
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "turtlesim");
        
        // 2. 初始化刷新背景的服务客户端
        clear_client_ = this->create_client<std_srvs::srv::Empty>("/clear");

        // 3. 定时器
        timer_ = this->create_wall_timer(100ms, std::bind(&ColorChanger::update_color, this));
    }

private:
    void update_color() {
        r_ = (r_ + 5) % 256;
        g_ = (g_ + 10) % 256;
        b_ = (b_ + 15) % 256;

        // 异步检查服务在不在，不在就跳过，不原地死等
        if (!param_client_->service_is_ready()) {
            return;
        }

        // 4. 【改动点】发送异步请求，不加 wait，不阻塞
        param_client_->set_parameters({
            rclcpp::Parameter("background_r", r_),
            rclcpp::Parameter("background_g", g_),
            rclcpp::Parameter("background_b", b_)
        });

        // 5. 刷新屏幕
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        clear_client_->async_send_request(request);
    }

    // 成员变量类型也要改
    rclcpp::AsyncParametersClient::SharedPtr param_client_; 
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int r_, g_, b_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // 这里正常的 spin 就行，不会报错了
    rclcpp::spin(std::make_shared<ColorChanger>());
    rclcpp::shutdown();
    return 0;
}