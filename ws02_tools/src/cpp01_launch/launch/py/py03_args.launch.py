# 参数设置：
# 参数设置主要涉及到参数的声明与调用两部分，
# 其中声明被封装为 launch.actions.DeclareLaunchArgument，调用则被封装为 launch.substitutions import LaunchConfiguration。

# 需求：启动turtlesim_node节点时，可以动态设置背景色。

from pkg_resources import declare_namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    decl_bg_r = DeclareLaunchArgument(name="background_r", default_value="255")     # name：参数名称，default_value：默认值。
    decl_bg_g = DeclareLaunchArgument(name="background_g", default_value="255")
    decl_bg_b = DeclareLaunchArgument(name="background_b", default_value="255")

    t2 = Node(package="turtlesim",
            executable="turtlesim_node",
            parameters=[{"background_r": LaunchConfiguration("background_r"), "background_g": LaunchConfiguration("background_g"), "background_b": LaunchConfiguration("background_b")}]
            )

    return LaunchDescription([decl_bg_r, decl_bg_g, decl_bg_b, t2])

# launch文件执行时，可以动态传入参数（不传参执行默认值），示例如下：
# ros2 launch cpp01_launch py03_args.launch.py background_r:=200 background_g:=80 background_b:=20
# 执行途中也可以用 rqt 改变颜色