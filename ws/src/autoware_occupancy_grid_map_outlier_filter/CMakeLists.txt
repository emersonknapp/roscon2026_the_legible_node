# Copyright 2021 Tier IV, Inc. All rights reserved.
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
cmake_minimum_required(VERSION 3.14)
project(autoware_occupancy_grid_map_outlier_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# -- Occupancy Grid Map Outlier Filter --
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent"
  EXECUTABLE ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  ament_add_gtest(test_pipeline_integration test/test_pipeline_integration.cpp)
  target_link_libraries(test_pipeline_integration ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
