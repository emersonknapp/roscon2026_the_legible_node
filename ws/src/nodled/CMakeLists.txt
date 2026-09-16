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
cmake_minimum_required(VERSION 3.22)
project(nodl_ogmof)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_nodl REQUIRED)
find_package(nodl_generator_cpp REQUIRED)
find_package(thin_ogmof REQUIRED)
ament_auto_find_build_dependencies()

##################
# NoDL registration
#
# Publish the node's interface (and its filter-parameter include) to the
# ament index so downstream tooling (`ros2 nodl ...`) and conformance tests
# can find it.  The `local://` include is rewritten to `nodl://` on install,
# so the sibling document must be registered in the same package.
ament_nodl_register(radius_search_2d_filter FILE nodl/radius_search_2d_filter.nodl.yaml)
ament_nodl_register(occupancy_grid_map_outlier_filter FILE nodl/node.nodl.yaml)

################
# Generated base
#
# Generate the abstract base class from the NoDL document.  Creates the STATIC
# library target `occupancy_grid_map_outlier_filter_base` exposing class
# `OccupancyGridMapOutlierFilterBase` (runtime node name
# `occupancy_grid_map_outlier_filter`).  It owns the parameters, publisher, and
# subscriptions; the subclass supplies only business logic.
nodl_generate_cpp(occupancy_grid_map_outlier_filter_base nodl/node.nodl.yaml)
# Linked into a SHARED component below, so it must be position independent.
set_target_properties(occupancy_grid_map_outlier_filter_base
  PROPERTIES POSITION_INDEPENDENT_CODE ON)

######
# Node

add_library(${PROJECT_NAME}_component SHARED
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_component
  PRIVATE
    occupancy_grid_map_outlier_filter_base
    thin_ogmof::thin_ogmof
    message_filters::message_filters
    rclcpp_components::component
    tf2_ros::tf2_ros
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "occupancy_grid_map_outlier_filter::NodlNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

##############
# Installation

# rclcpp_components_register_node() installs the generated executable under
# lib/${PROJECT_NAME}; only the component library needs an explicit rule here.
install(
  TARGETS ${PROJECT_NAME}_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

#######
# Tests

if(BUILD_TESTING)
  # Smoke test: launch the node and assert its live interface matches the NoDL
  # document (base-class infrastructure endpoints are stripped automatically).
  nodl_add_conformance_test(occupancy_grid_map_outlier_filter_conformance
    EXECUTABLE ${PROJECT_NAME}_node
    NODL_FILE nodl/node.nodl.yaml
    NODE_NAME occupancy_grid_map_outlier_filter
  )
endif()

ament_package()
