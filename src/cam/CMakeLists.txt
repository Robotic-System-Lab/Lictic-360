cmake_minimum_required(VERSION 3.8)
project(cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)
include_directories(${CUDA_INCLUDE_DIRS})
add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})
# Misalnya, jika image_converter dan jetson-utils merupakan library eksternal,
# tambahkan find_package atau include_directories yang dibutuhkan.

# Build executable
add_executable(video_source src/node_video_source.cpp)
ament_target_dependencies(video_source rclcpp sensor_msgs)

# Install executable
install(TARGETS
  video_source
  DESTINATION lib/${PROJECT_NAME})

# Install file launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()