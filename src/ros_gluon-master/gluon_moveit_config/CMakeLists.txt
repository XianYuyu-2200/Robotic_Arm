cmake_minimum_required(VERSION 3.8)
project(gluon_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(move_group_demo src/move_group_demo.cpp)
ament_target_dependencies(move_group_demo rclcpp moveit_ros_planning_interface)

add_executable(target_approach_node src/target_approach_node.cpp)
ament_target_dependencies(target_approach_node
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_geometry
  message_filters
  moveit_msgs
  moveit_ros_planning_interface
)
target_link_libraries(target_approach_node ${OpenCV_LIBS})

install(TARGETS move_group_demo target_approach_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
