/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/

#include "rclcpp/rclcpp.hpp"

class MinimalParamServer: public rclcpp::Node{
    public:
		MinimalParamServer()
      : Node("minimal_param_server", rclcpp::NodeOptions().allow_undeclared_parameters(true))
      {}

      //参数声明
      void declare_param()
      {
          //声明参数并设置默认值
          this->declare_parameter("car_type", "Tiger");
          this->declare_parameter("height", 1.50);
          this->declare_parameter("wheels", 4);
          // 需要设置 rclcpp::NodeOptions().allow_undeclared_parameters(true),否则非法 
          this->set_parameter(rclcpp::Parameter("undcl_test", 100));
      }

			//查询参数
			void get_param()
			{
				RCLCPP_INFO(this->get_logger(), "------------------查----------------");
				//获取指定
				rclcpp::Parameter car_type = this->get_parameter("car_type");
				RCLCPP_INFO(this->get_logger(), "car_type:%s", car_type.as_string().c_str());
				RCLCPP_INFO(this->get_logger(), "height:%.2f", this->get_parameter("height").as_double());
				RCLCPP_INFO(this->get_logger(), "wheels:%ld", this->get_parameter("wheels").as_int());
				RCLCPP_INFO(this->get_logger(), "undcl_test:%ld", this->get_parameter("undcl_test").as_int());
				//判断包含
				RCLCPP_INFO(this->get_logger(), "包含car_type? %d", this->has_parameter("car_type"));
				RCLCPP_INFO(this->get_logger(), "包含car_typesxxxx? %d", this->has_parameter("car_typesxxxx"));
				//获取所有
				auto params = this->get_parameters({"car_type", "height", "wheels"});
				for(auto &param : params)
				{
					RCLCPP_INFO(this->get_logger(), "name = %s, value = %s", param.get_name().c_str(), param.value_to_string().c_str());

				}
			}

			//修改参数
			void update_param()
			{
				RCLCPP_INFO(this->get_logger(), "------------------改----------------");
				this->set_parameter(rclcpp::Parameter("height", 1.75));
				RCLCPP_INFO(this->get_logger(),"删除操作前, undcl_test还存在吗? %d", this->has_parameter("undcl_test"));
				this->undeclare_parameter("undcl_test");
				RCLCPP_INFO(this->get_logger(),"删除操作前, undcl_test还存在吗? %d", this->has_parameter("undcl_test"));
			}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
	auto paramServer = std::make_shared<MinimalParamServer>();
	paramServer->declare_param();
	paramServer->get_param();
	paramServer->update_param();
	rclcpp::spin(paramServer);

	rclcpp::shutdown();
	return 0;
}