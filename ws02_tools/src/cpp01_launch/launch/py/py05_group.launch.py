# 分组设置：
# 在 launch 文件中，为了方便管理可以对节点分组，分组相关API为：launch.actions.GroupAction和launch_ros.actions.PushRosNamespace。



from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():

    turtle1 = Node(package="turtlesim", executable="turtlesim_node", name="t1")
    turtle2 = Node(package="turtlesim", executable="turtlesim_node", name="t2")
    turtle3 = Node(package="turtlesim", executable="turtlesim_node", name="t3")

    g1 = GroupAction(actions=[PushRosNamespace(namespace="g1"), turtle1, turtle2])  # actions：action列表，比如被包含到组内的命名空间、节点等
    g2 = GroupAction(actions=[PushRosNamespace(namespace="g2"), turtle3])           # namespace：当前组使用的命名空间
    
    return LaunchDescription([g1, g2])