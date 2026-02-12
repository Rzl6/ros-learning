from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():

    turtle1 = Node(package="turtlesim", executable="turtlesim_node");
    spawn = ExecuteProcess(
        # cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{x: 8.0, y: 9.0,theta: 0.0, name: 'turtle2'}\""],
        # 或
        cmd= [
            FindExecutable(name="ros2"), # 不可以有空格
            " service call",    #在 Launch 文件里，如果你把命令拆成列表，系统最后会把它们拼接起来,所以从第二个变量开始，前面要有空格
            " /spawn turtlesim/srv/Spawn",
            " \"{x: 8.0, y: 9.0, theta: 1.0, name: 'turtle2'}\" "
        ],
        output="both",
        shell=True
    )

    return LaunchDescription([turtle1, spawn])
    