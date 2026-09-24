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
project(thin_ogmof)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(NOT APPLE)
    add_link_options("-Wl,--no-undefined")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

##############
# Core Library

add_library(${PROJECT_NAME} SHARED
  src/outlier_filter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    pcl_ros::pcl_ros_tf
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    tf2_ros::tf2_ros
  PRIVATE
    tf2_eigen::tf2_eigen
)

######
# Node

add_library(${PROJECT_NAME}_component SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
    ${PROJECT_NAME}
    message_filters::message_filters
    nav_msgs::nav_msgs
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    tf2_ros::tf2_ros
  PRIVATE
    rclcpp_components::component
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "occupancy_grid_map_outlier_filter::Node"
  EXECUTABLE ${PROJECT_NAME}_node
)

##############
# Installation

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_component
    ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}_TARGETS
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  EXPORT ${PROJECT_NAME}_TARGETS
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)

#######
# Tests

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_outlier_filter test/test_outlier_filter.cpp)
  target_link_libraries(test_outlier_filter ${PROJECT_NAME})
endif()

ament_package()
