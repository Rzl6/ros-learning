# 文件包含：
# 在 launch 文件中可以包含其他launch文件，
# 需要使用的API为：launch.actions.IncludeLaunchDescription 和 launch.launch_description_sources.PythonLaunchDescriptionSource。

# 下面代码将包含一个launch文件并为launch文件传参

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    include_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(  # 被包含的 launch 文件路径
                get_package_share_directory("cpp01_launch"), # 自动寻找目录
                "launch/py",
                "py03_args.launch.py"
            )
        ),                          # 用于设置被包含的 launch 文件     
        launch_arguments={
            "background_r": "200",
            "background_g": "100",
            "background_b": "70",
        }.items()                   # 元组列表，每个元组中都包含参数的键和值
    )

    return LaunchDescription([include_launch])