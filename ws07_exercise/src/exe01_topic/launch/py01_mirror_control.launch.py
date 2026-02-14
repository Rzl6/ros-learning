from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # 1. 启动乌龟1 (namespace 为 t1)
    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t1", # 专业写法：直接用 namespace 参数
        name="turtlesim"
    )

    # 2. 启动乌龟2 (namespace 为 t2)
    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t2",
        name="turtlesim"
    )

    # 3. 启动你的镜像控制节点
    # 可执行文件名是 ex02_mirror_control
    mirror_node = Node(
        package="exe01_topic",
        executable="ex02_mirror_control", # 确保这里和你 CMakeLists.txt 里写的一致
        output="screen"
    )

    # 注意：我删掉了键盘控制节点，建议你手动在另一个终端运行
    return LaunchDescription([
        turtle1,
        turtle2,
        mirror_node
    ])