cmake_minimum_required(VERSION 3.16)
project(vectornav_imu_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Set the path to VectorNav SDK (adjust this path as needed)
set(VECTORNAV_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vnsdk-1-0-0/cpp")

# Include VectorNav SDK headers
include_directories(
  include
  ${VECTORNAV_SDK_PATH}/include
)

# Add VectorNav SDK library
add_subdirectory(${VECTORNAV_SDK_PATH}/src vectornav_sdk)

# Add executable
add_executable(vectornav_imu_node
  src/vectornav_imu_node.cpp
  src/vectornav_imu_publisher.cpp
)

target_include_directories(vectornav_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
ament_target_dependencies(vectornav_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(vectornav_imu_node
  VnSensor
)

# Set RPATH so the executable can find libVnSensor.so in the same directory
set_target_properties(vectornav_imu_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH FALSE
)

# Install targets
install(TARGETS vectornav_imu_node VnSensor
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()