# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.8)
project(rov_physics_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Vehicle and pool model generation, shared by the launch file, the tools and the tests.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/hydrostatics scripts/trim_check
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py
    PYTHON_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
