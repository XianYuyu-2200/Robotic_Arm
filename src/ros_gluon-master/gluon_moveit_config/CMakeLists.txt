cmake_minimum_required(VERSION 3.8)
project(gluon_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_group_demo src/move_group_demo.cpp)
ament_target_dependencies(move_group_demo rclcpp moveit_ros_planning_interface)

install(TARGETS move_group_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
