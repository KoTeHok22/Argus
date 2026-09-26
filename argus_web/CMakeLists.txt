cmake_minimum_required(VERSION 3.8)
project(argus_web)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  argus_web/server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY argus_web static
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(argus_web_pytest test)
endif()

ament_package()
