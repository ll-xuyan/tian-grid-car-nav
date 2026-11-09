import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_gazebo')

    # 1. 启动田字格世界（gzserver + gzclient）
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'square_10m.launch.py')))

    # 2. 机器人模型描述
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 3. robot_state_publisher：发布雷达等固定坐标系的 TF
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}],
               output='screen')

    # 4. 延迟 10 秒生成小车（等 Gazebo 世界加载完）
    spawn = Node(package='gazebo_ros',
                 executable='spawn_entity.py',
                 arguments=['-entity', 'my_car', '-file', urdf_file,
                            '-x', '0', '-y', '0', '-z', '0.05'],
                 output='screen')

    spawn_delayed = TimerAction(period=10.0, actions=[spawn])

    return LaunchDescription([world_launch, rsp, spawn_delayed])
