from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # 1. 启动乌龟
    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    distance_server = Node(
        package="exe02_service",
        executable="ex01_distance_server",
        output="screen"
    )

    return LaunchDescription([turtle1, distance_server])