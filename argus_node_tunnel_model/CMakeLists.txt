
cmake_minimum_required(VERSION 3.8)
project(argus_node_tunnel_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(argus_core REQUIRED)
find_package(argus_msgs REQUIRED)

add_executable(argus_tunnel_model_node src/argus_tunnel_model_node.cpp)
target_compile_features(argus_tunnel_model_node PUBLIC cxx_std_17)
ament_target_dependencies(argus_tunnel_model_node
  rclcpp sensor_msgs std_msgs argus_core argus_msgs)

install(TARGETS argus_tunnel_model_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
