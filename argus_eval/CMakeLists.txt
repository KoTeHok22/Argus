cmake_minimum_required(VERSION 3.8)
project(argus_eval)

find_package(ament_cmake REQUIRED)

# Python-модуль устанавливается скриптом setup.py; CMake ставит только entry point.
install(PROGRAMS
  argus_eval/run_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY argus_eval
  DESTINATION lib/python3/dist/argus_eval_pkg
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
