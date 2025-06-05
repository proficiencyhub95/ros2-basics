cmake_minimum_required(VERSION 3.5)
project(custom_pkg)

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
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(pub_cpp src/publisher.cpp)
add_executable(sub_cpp src/subscriber.cpp)
add_executable(server_cpp src/server.cpp)
add_executable(client_cpp src/client.cpp)

ament_target_dependencies(pub_cpp rclcpp std_msgs)
ament_target_dependencies(sub_cpp rclcpp std_msgs)
ament_target_dependencies(server_cpp rclcpp example_interfaces)
ament_target_dependencies(client_cpp rclcpp example_interfaces)

rosidl_generate_interfaces(
${PROJECT_NAME}
"action/Fibonacci.action"
)

install(
TARGETS pub_cpp sub_cpp server_cpp client_cpp
DESTINATION lib/${PROJECT_NAME}
)

install(
DIRECTORY launch 
DESTINATION share/${PROJECT_NAME}
)

ament_package()
