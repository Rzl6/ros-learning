# 执行指令：
# 下面代码用于执行 cmd 参数中的命令，该命令会在 turtlesim_node 中生成一只新的小乌龟。
# 运行 turtlesim_node 时，它不仅打开了一个蓝色的窗口，还在后台悄悄开启了一个名叫 /spawn 的窗口，下面定义的 spawn 是基于此运行的

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():

    turtle1 = Node(package="turtlesim", executable="turtlesim_node")    # 乌龟1
    spawn = ExecuteProcess(
        # cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{x: 8.0, y: 9.0,theta: 0.0, name: 'turtle2'}\""],
        # 或
        cmd= [                              # 被执行的命令
            FindExecutable(name="ros2"),    # 不可以有空格
            " service call",                # 在 Launch 文件里，如果你把命令拆成列表，系统最后会把它们拼接起来,所以从第二个变量开始，前面要有空格
            " /spawn turtlesim/srv/Spawn",
            " \"{x: 8.0, y: 9.0, theta: 1.0, name: 'turtle2'}\" "
        ],                                  
        output="both",                      # 设置为 both 时，日志会被输出到日志文件和终端，默认为 log，日志只输出到日志文件
        shell=True                          # 如果为 True，则以 shell 的方式执行命令
    )   #乌龟2

    return LaunchDescription([turtle1, spawn])
    