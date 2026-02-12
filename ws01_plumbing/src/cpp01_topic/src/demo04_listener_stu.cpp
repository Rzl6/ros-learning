/*  
    需求：订阅发布方发布的学生消息，并输出到终端。
*/

//包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using std::placeholders::_1;
using base_interfaces_demo::msg::Student;

//定义节点类
class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber()
        : Node("student_subscriber")
        {
            //创建订阅方
            subscription_ = this->create_subscription<Student>("topic_stu", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }
    private:
        void topic_callback(const Student & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "订阅的学生消息: name=%s, age=%d, height=%.2f", msg.name.c_str(), msg.age, msg.height);
        }    
        rclcpp::Subscription<Student>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}

