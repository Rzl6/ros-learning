/*  
  需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。
*/

//包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::msg::Student;

//定义节点类
class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher()
        : Node("student_publisher"), count_(0)
        {
          //创建发布方
          publisher_ = this->create_publisher<Student>("topic_stu", 10);
          //创建定时器
          timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
          //组织消息并发布
          auto stu = Student();
          stu.name = "zhang";
          stu.age = count_++;
          stu.height = 1.65;
          RCLCPP_INFO(this->get_logger(), "学生信息: name=%s, age=%d, height=%.2f", stu.name.c_str(), stu.age, stu.height);
          publisher_->publish(stu);
        }
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<Student>::SharedPtr publisher_;

};

int main(int argc, char *argv[])
{
  //初始化客户端
  rclcpp::init(argc, argv);
  //调用spin函数传入节点
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  //释放资源
  rclcpp::shutdown();
  return 0;
}