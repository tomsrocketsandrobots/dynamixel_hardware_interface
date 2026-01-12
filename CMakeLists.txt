################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(dynamixel_sdk REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynamixel_interfaces REQUIRED)

################################################################################
# Build
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamixelCommand.msg"
  DEPENDENCIES std_msgs
)

add_library(
  ${PROJECT_NAME}_lib
  SHARED
  src/dynamixel_hardware_interface.cpp
  src/dynamixel/dynamixel_info.cpp
  src/dynamixel/dynamixel.cpp
)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_lib
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${realtime_tools_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${transmission_interface_INCLUDE_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
  ${hardware_interface_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${dynamixel_sdk_LIBRARIES}
  ${std_srvs_TARGETS}
  ${dynamixel_interfaces_TARGETS}
  ${realtime_tools_TARGETS}
  ${transmission_interface_TARGETS}
  ${std_msgs_TARGETS}
  ${cpp_typesupport_target}
)

pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware_interface_plugin.xml)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/create_udev_rules
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  dynamixel_sdk
  dynamixel_interfaces
  transmission_interface
  std_msgs
  rosidl_default_runtime
)

ament_package()
