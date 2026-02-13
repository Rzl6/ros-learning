# 添加事件：
# 节点在运行过程中会触发不同的事件，当事件触发时可以为之注册一定的处理逻辑。
# 事件使用相关的 API 为：launch.actions.RegisterEventHandler、launch.event_handlers.OnProcessStart、launch.event_handlers.OnProcessExit。

# 为 turtlesim_node 节点添加事件，事件1：节点启动时调用spawn服务生成新乌龟；事件2：节点关闭时，输出日志信息。

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import FindExecutable
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():

    turtle = Node(package="turtlesim", executable="turtlesim_node")
    spawn = ExecuteProcess(
        cmd=[
            FindExecutable(name= "ros2"),
            " service call",
            " /spawn turtlesim/srv/Spawn",
            '\"{x: 8.0, y: 1.0, theta: 1.0, name: "turtle2"}'
        ],
        output="both",
        shell=True
    )

    start_event = RegisterEventHandler(     # OnProcessStart 是启动事件对象
        event_handler=OnProcessStart(       # event_handler：注册的事件对象
            target_action=turtle,           # target_action：被注册事件的目标对象，on_start：事件触发时的执行逻辑。
            on_start= spawn
        )
    )   

    exit_event = RegisterEventHandler(      # OnProcessExit 是退出事件对象
        event_handler=OnProcessExit(        # target_action：被注册事件的目标对象，on_exit：事件触发时的执行逻辑。
            target_action= turtle,
            on_exit= [LogInfo(msg="turtlesim_node退出!")]   # LogInfo 是日志输出对象，其参数为：msg：被输出的日志信息。
        )
    )

    return LaunchDescription([turtle, start_event, exit_event])