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

include_directories(
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/occupancy_grid_map_outlier_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# -- Occupancy Grid Map Outlier Filter --
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent"
  EXECUTABLE ${PROJECT_NAME}_node)

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
