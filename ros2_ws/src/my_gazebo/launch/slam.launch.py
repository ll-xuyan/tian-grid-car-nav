import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_gazebo')
    slam_share = get_package_share_directory('slam_toolbox')

    # 1. slam_toolbox 在线建图（用我们自己的参数）
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_params.yaml'),
            'use_sim_time': 'true'}.items())

    # 2. RViz：实时显示正在构建的地图（用我们自己的完整配置）
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'config', 'slam_view.rviz')],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([slam, rviz])
