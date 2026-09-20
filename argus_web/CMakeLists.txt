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

ament_package()
