cmake_minimum_required(VERSION 3.8)
project(gluon_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
	set(BUILD_PLATFORM linux_x86_64)
else()
	set(BUILD_PLATFORM windows_x64)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../gluon/ActuatorController_SDK/sdk/include
)

add_library(gluon_control SHARED
  src/gluon_hw_interface.cpp
)

target_include_directories(gluon_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gluon_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

# Link SDK
if(WIN32)
  set(LIB_EXT "lib")
  set(LIB_PREFIX "")
else()
  set(LIB_EXT "so")
  set(LIB_PREFIX "lib")
endif()

# Note: The SDK path is relative to the 'gluon' package, which is a sibling.
# We assume standard workspace layout.
target_link_libraries(gluon_control
   ${CMAKE_CURRENT_SOURCE_DIR}/../gluon/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/${LIB_PREFIX}ActuatorController.${LIB_EXT}
)

# Register the plugin
pluginlib_export_plugin_description_file(hardware_interface gluon_control_plugins.xml)

install(TARGETS gluon_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES gluon_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
