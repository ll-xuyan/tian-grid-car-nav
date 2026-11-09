import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('my_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_share, 'worlds', 'square_10m.world')

    # Gazebo 服务器：加载自定义世界
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items())

    # Gazebo 图形界面
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')))

    return LaunchDescription([gzserver, gzclient])
