# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Сборка пакета Argus.

cmake_minimum_required(VERSION 3.8)
project(argus_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cloud_filter.cpp
  src/range_image.cpp
  src/gauge.cpp
  src/clustering.cpp
  src/tracking.cpp
  src/explain.cpp
  src/anomaly.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
# Только msg-типы sensor_msgs (заголовки, без линковки и без rclcpp) —
# ядро остаётся тестируемым вне ROS-рантайма (T5 в уточнённой формулировке).

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 PCL sensor_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_cloud_filter test/test_cloud_filter.cpp)
  target_link_libraries(test_cloud_filter ${PROJECT_NAME})
  ament_add_gtest(test_range_image test/test_range_image.cpp)
  target_link_libraries(test_range_image ${PROJECT_NAME})
  ament_add_gtest(test_gauge test/test_gauge.cpp)
  target_link_libraries(test_gauge ${PROJECT_NAME})
  ament_add_gtest(test_clustering test/test_clustering.cpp)
  target_link_libraries(test_clustering ${PROJECT_NAME})
  ament_add_gtest(test_tracking test/test_tracking.cpp)
  target_link_libraries(test_tracking ${PROJECT_NAME})
endif()

ament_package()
