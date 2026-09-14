# Copyright 2021 Tier IV, Inc. All rights reserved.
# Copyright 2026 Alistair English, Emerson Knapp
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
project(thin_occupancy_grid_map_outlier_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

# -- Core business logic (no rclcpp / comms) --
# Private, in-package library. Its headers live beside their sources under
# src/outlier_filter and are exposed only in the BUILD interface, so they are
# never installed or exported as public API.
ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/outlier_filter/point_cloud_util.cpp
  src/outlier_filter/occupancy_grid.cpp
  src/outlier_filter/pipeline.cpp
  src/outlier_filter/outlier_filter.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core
  ${PCL_LIBRARIES}
)

# -- Thin node --
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
# target_link_libraries(${PROJECT_NAME}
#   ${PROJECT_NAME}_core
#   ${PCL_LIBRARIES}
# )
# Mark this target as the one building the public library, so the visibility
# macros export (rather than import) the public symbols.
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${PCL_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "THIN_OGM_OUTLIER_FILTER_BUILDING_LIBRARY")

# -- Occupancy Grid Map Outlier Filter --
# Runtime-by-name / dlopen / launch composition. The plugin string must match
# the C++ namespace of the class (thin_occupancy_grid_map_outlier_filter).
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "thin_occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent"
  EXECUTABLE ${PROJECT_NAME}_node)

# ament_auto_add_library already installs and exports ${PROJECT_NAME} as a
# target, and ament_auto_package installs the include/ directory -- together
# these give downstream packages link-time composition via the public factory
# header, while src/ headers stay private (build-interface only).

ament_auto_package()
