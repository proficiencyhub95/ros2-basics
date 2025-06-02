cmake_minimum_required(VERSION 3.5)
project(day-1)

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

add_executable(pub_cpp src/publisher.cpp)
add_executable(sub_cpp src/subscriber.cpp)

ament_target_dependencies(pub_cpp rclcpp std_msgs)
ament_target_dependencies(sub_cpp rclcpp std_msgs)

install(
TARGETS pub_cpp sub_cpp
DESTINATION lib/${PROJECT_NAME}
)

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
