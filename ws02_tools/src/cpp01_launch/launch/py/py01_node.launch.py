from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    turtle1 = Node(package="turtlesim",
                executable="turtlesim_node",
                namespace="ns_1",
                name="t1",
                exec_name="turtle_label",
                respawn= True
                )
    
    turtle2 = Node(package="turtlesim",
                executable="turtlesim_node",
                name="t2",
                # 参数设置方式1
                # parameters=[{"background_r": 0,"background_g": 0,"background_b": 0}],
                # 参数设置方式2: 从 yaml 文件加载参数，yaml 文件所属目录需要在配置文件中安装。
                parameters=[os.path.join(get_package_share_directory("cpp01_launch"),"config","t2.yaml")]
                )
    
    turtle3 = Node(package="turtlesim",
                executable="turtlesim_node",
                name="t3",
                remappings=[("/turtle1/cmd_vel", "/cmd_vel")]   #话题重映射
                )
    
    turtle4 = Node(package="turtlesim",
                executable="turtlesim_node",
                # 节点启动时传参，相当于 arguments 传参时添加前缀 --ros-args 
                ros_arguments=["--remap", "__ns:=/t4_ns", "--remap", "__node:=t4"]
                )
    
    rviz = Node(package="rviz2",
                executable="rviz2",
                #节点启动的时候传参
                arguments=["-d", os.path.join(get_package_share_directory("cpp01_launch"), "config", "my.rviz")]
                )
    
    return LaunchDescription([turtle1, turtle2, turtle3, rviz, turtle4])