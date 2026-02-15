from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    turtle = Node(package="turtlesim", executable="turtlesim_node")
    
    x_arg = DeclareLaunchArgument("x", default_value="2.0")
    y_arg = DeclareLaunchArgument("y", default_value="2.0")

    distance_server = Node(
        package="exe03_action",
        executable="ex01_nav_server",
        output = "screen"
    )

    distance_client = Node(
        package="exe03_action",
        executable="ex02_nav_client",
        arguments=[LaunchConfiguration("x"), LaunchConfiguration("y")],
        output = "screen"
    )

    return LaunchDescription([x_arg, y_arg, turtle, distance_server, distance_client])