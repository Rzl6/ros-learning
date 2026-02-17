from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    turtle1 = Node(package="turtlesim", executable="turtlesim_node", name="turtlesim")
    param_client = Node(package="exe04_param", executable="ex01_param_client", output="screen")
    return LaunchDescription([turtle1, param_client])