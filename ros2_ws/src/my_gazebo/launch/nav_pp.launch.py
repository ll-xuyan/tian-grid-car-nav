import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """导航（方案B：Pure Pursuit）+ RViz 导航界面。
    需先启动仿真: ~/start_car.sh
    使用: ros2 launch my_gazebo nav_pp.launch.py
    """
    pkg_share = get_package_share_directory('my_gazebo')
    rviz_cfg = os.path.join(pkg_share, 'config', 'nav_view.rviz')

    nav_node = Node(
        package='my_gazebo',
        executable='pure_pursuit_nav.py',
        name='pure_pursuit_nav',
        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen')

    return LaunchDescription([nav_node, rviz_node])
