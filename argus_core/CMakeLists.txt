
cmake_minimum_required(VERSION 3.8)
project(argus_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PWP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/patchworkpp")
if(NOT EXISTS "${PWP_ROOT}/cpp/patchworkpp/src/patchworkpp.cpp")
  message(FATAL_ERROR
    "Patchwork++ не найден в ${PWP_ROOT}. "
    "Импорт: vcs import third_party < third_party/argus.repos")
endif()
set(PARENT_PROJECT_NAME patchworkpp)
set(TARGET_NAME ground_seg_cores)
add_subdirectory("${PWP_ROOT}/cpp/common" pwp_common)
add_subdirectory("${PWP_ROOT}/cpp/patchworkpp" pwp_pp)

add_library(${PROJECT_NAME} SHARED
  src/cloud_filter.cpp
  src/range_image.cpp
  src/gauge.cpp
  src/clustering.cpp
  src/tracking.cpp
  src/fusion.cpp
  src/explain.cpp
  src/anomaly_no_return.cpp
  src/anomaly_geometry.cpp
  src/anomaly_temporal.cpp
  src/anomaly_free_space.cpp
  src/ground.cpp
  src/odometry.cpp
  src/tunnel_model.cpp
  src/tunnel_profile.cpp
  src/track_corridor.cpp
  src/frame_sync.cpp
  src/health_monitor.cpp
  src/sensor_health.cpp
  src/supervision.cpp
  src/classification.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen ground_seg_cores)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  "${PWP_ROOT}/cpp/patchworkpp/include"
  "${PWP_ROOT}/cpp/common/include")
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(offline_detector tools/offline_detector.cpp)
target_link_libraries(offline_detector PRIVATE ${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 PCL sensor_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} ground_seg_cores ground_seg_common
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS offline_detector
  RUNTIME DESTINATION lib/argus_core
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_cloud_filter test/test_cloud_filter.cpp)
  target_link_libraries(test_cloud_filter ${PROJECT_NAME})
  ament_add_gtest(test_range_image test/test_range_image.cpp)
  target_link_libraries(test_range_image ${PROJECT_NAME})
  ament_add_gtest(test_anomaly test/test_anomaly.cpp)
  target_link_libraries(test_anomaly ${PROJECT_NAME})
  ament_add_gtest(test_gauge test/test_gauge.cpp)
  target_link_libraries(test_gauge ${PROJECT_NAME})
  ament_add_gtest(test_clustering test/test_clustering.cpp)
  target_link_libraries(test_clustering ${PROJECT_NAME})
  ament_add_gtest(test_fusion test/test_fusion.cpp)
  target_link_libraries(test_fusion ${PROJECT_NAME})
  ament_add_gtest(test_tracking test/test_tracking.cpp)
  target_link_libraries(test_tracking ${PROJECT_NAME})
  ament_add_gtest(test_ground test/test_ground.cpp)
  target_link_libraries(test_ground ${PROJECT_NAME})
  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry ${PROJECT_NAME})
  ament_add_gtest(test_tunnel_model test/test_tunnel_model.cpp)
  target_link_libraries(test_tunnel_model ${PROJECT_NAME})
  ament_add_gtest(test_tunnel_profile test/test_tunnel_profile.cpp)
  target_link_libraries(test_tunnel_profile ${PROJECT_NAME})
  ament_add_gtest(test_track_corridor test/test_track_corridor.cpp)
  target_link_libraries(test_track_corridor ${PROJECT_NAME})
  ament_add_gtest(test_frame_sync test/test_frame_sync.cpp)
  target_link_libraries(test_frame_sync ${PROJECT_NAME})
  ament_add_gtest(test_health_monitor test/test_health_monitor.cpp)
  target_link_libraries(test_health_monitor ${PROJECT_NAME})
  ament_add_gtest(test_sensor_health test/test_sensor_health.cpp)
  target_link_libraries(test_sensor_health ${PROJECT_NAME})
  ament_add_gtest(test_supervision test/test_supervision.cpp)
  target_link_libraries(test_supervision ${PROJECT_NAME})
  ament_add_gtest(test_classification test/test_classification.cpp)
  target_link_libraries(test_classification ${PROJECT_NAME})
endif()

ament_package()
