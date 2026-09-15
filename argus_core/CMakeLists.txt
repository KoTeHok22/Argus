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
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

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
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

ament_target_dependencies(${PROJECT_NAME} Eigen3)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)

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
