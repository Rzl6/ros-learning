/*  
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/

//包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

//定义节点类
class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber()
        : Node("minimal_subscriber")
        {
            //创建订阅方
            subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }

    private:
        //处理订阅到的消息
        void topic_callback(const std_msgs::msg::String & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "订阅的消息：'%s'",msg.data.c_str());

        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

int main(int argc, char *argv[])
{
    //初始化ROS2客户端
    rclcpp::init(argc, argv);
    //调用spin函数，传入节点对象指针
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    //释放资源
    rclcpp::shutdown();
    return 0;
}