cmake_minimum_required(VERSION 3.5)
project(odom_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)


add_executable(odom_converter src/odom_converter.cpp)
ament_target_dependencies(odom_converter rclcpp nav_msgs tf2)


install(TARGETS
  odom_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
