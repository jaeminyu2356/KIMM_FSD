#  Copyright (c) 2022 Jonas Mahler

#  This file is part of pcl_example.

#  pcl_example is free software: you can redistribute it and/or modify it under the terms 
#  of the GNU General Public License as published by the Free Software Foundation, 
#  either version 3 of the License, or (at your option) any later version.

#  pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along 
#  with pcl_example. If not, see <https://www.gnu.org/licenses/>. 

cmake_minimum_required(VERSION 3.5)
project(pcd_cal)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_to_pcl_parser src/map_to_pcl_parser.cpp)
ament_target_dependencies(
  map_to_pcl_parser 
  rclcpp std_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
)
target_link_libraries( map_to_pcl_parser
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

add_executable(map_to_pcl_parser_sim src/map_to_pcl_parser_sim.cpp)
ament_target_dependencies(
  map_to_pcl_parser_sim 
  rclcpp std_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
)
target_link_libraries( map_to_pcl_parser_sim
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

add_executable(pcd_visualization src/pcd_visualization.cpp)
ament_target_dependencies(
  pcd_visualization 
  rclcpp std_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
)
target_link_libraries( pcd_visualization
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

add_executable(laser_scan src/laser_scan.cpp)
ament_target_dependencies(
  laser_scan 
  rclcpp std_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
)
target_link_libraries( laser_scan
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

install(TARGETS
  map_to_pcl_parser
  map_to_pcl_parser_sim
  pcd_visualization
  laser_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config launch src srv rviz
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
