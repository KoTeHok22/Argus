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

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(argus_eval_pytest test)
endif()

ament_package()
