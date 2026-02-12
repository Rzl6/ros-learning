/*
    需求：编写参数客户端，获取或修改服务端参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.查询参数；
            3-2.修改参数；
        4.创建节点对象指针，调用参数操作函数；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalParamClient: public rclcpp::Node{
    public:
        MinimalParamClient()
        : Node("paramDemoClient_node")
        {
            paramClient = std::make_shared<rclcpp::SyncParametersClient>(this, "minimal_param_server");
        }
        
        bool connect_server()
        {
            //等待服务连接
            while(!paramClient->wait_for_service(1s)){
                if(!rclcpp::ok()){
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "服务未连接");

            }
            return true;
        }

        //查询参数
        void get_param()
        {
            RCLCPP_INFO(this->get_logger(), "-----------参数客户端查询参数-----------");
            double height = paramClient->get_parameter<double>("height");
            RCLCPP_INFO(this->get_logger(), "height = %.2f", height);
            RCLCPP_INFO(this->get_logger(), "car_type存在吗? %d", paramClient->has_parameter("car_type"));
            auto params = paramClient->get_parameters({"car_type", "height", "wheels"});
            for(auto &param : params){
                RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(), param.value_to_string().c_str());
            }

        }

        //修改参数
        void update_param()
        {
            RCLCPP_INFO(this->get_logger(), "-----------参数客户端修改参数-----------");
            paramClient->set_parameters({rclcpp::Parameter("car_type", "Mouse"),
                rclcpp::Parameter("height", 2.0),
                rclcpp::Parameter("wheels", 6),
                rclcpp::Parameter("width", 0.15)
            });
        }
    private:
        rclcpp::SyncParametersClient::SharedPtr paramClient;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto paraClient = std::make_shared<MinimalParamClient>();
    bool flag = paraClient->connect_server();
    if(!flag){
        return 0;
    }
    paraClient->get_param();
    paraClient->update_param();
    paraClient->get_param();

    rclcpp::shutdown();
    return 0;
}
