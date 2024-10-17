cmake_minimum_required(VERSION 3.5)
project(lamport_clks)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_msgs REQUIRED)

add_executable(sample_node src/sample_node.cpp)
ament_target_dependencies(sample_node rclcpp test_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  sample_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()