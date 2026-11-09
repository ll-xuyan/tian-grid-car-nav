cmake_minimum_required(VERSION 3.8)
project(my_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds urdf config
  DESTINATION share/${PROJECT_NAME}
)

# 方案B导航节点（Pure Pursuit 循迹）
install(PROGRAMS
  scripts/pure_pursuit_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
