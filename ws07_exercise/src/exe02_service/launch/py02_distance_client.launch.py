from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    x_arg = DeclareLaunchArgument("x", default_value="2.0")
    y_arg = DeclareLaunchArgument("y", default_value="2.0")
    
    distance_client = Node(
        package="exe02_service",
        executable="ex02_distance_client",
        arguments=[LaunchConfiguration("x"), LaunchConfiguration("y")],
        output = "screen"
    )

    return LaunchDescription([x_arg, y_arg, distance_client])