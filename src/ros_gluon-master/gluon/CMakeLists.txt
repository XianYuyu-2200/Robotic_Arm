cmake_minimum_required(VERSION 3.8)
project(gluon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
	set(BUILD_PLATFORM
		linux_x86_64)
else()
	set(BUILD_PLATFORM
		windows_x64)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/include
)

add_executable(gluon_node src/gluonControl.cpp)
ament_target_dependencies(gluon_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Check for Windows vs Linux library extension
if(WIN32)
  set(LIB_EXT "lib")
  set(LIB_PREFIX "")
else()
  set(LIB_EXT "so")
  set(LIB_PREFIX "lib")
endif()

# Fallback to original path if not found, but try to handle it gracefully
# Assuming the original path structure:
target_link_libraries(gluon_node
   ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/${LIB_PREFIX}ActuatorController.${LIB_EXT}
)

install(TARGETS gluon_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
