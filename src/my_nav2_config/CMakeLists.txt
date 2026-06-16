cmake_minimum_required(VERSION 3.8)
project(my_nav2_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/waypoint_patrol.py
  scripts/waypoint_commint_patrol.py
  scripts/waypoint_commint_patrol_v2.py
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
