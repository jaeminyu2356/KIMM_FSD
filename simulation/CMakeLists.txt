cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(interactive_markers REQUIRED)

# C++ 실행 파일 추가
add_executable(kimm_city_node src/kimm_city_node.cpp)
ament_target_dependencies(kimm_city_node
  rclcpp
  gazebo_ros
  gazebo_msgs
)

add_library(polygon_costmap_layer SHARED
  src/polygon_costmap_layer.cpp
)

target_include_directories(polygon_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(polygon_costmap_layer
  "nav2_costmap_2d"
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
)

target_link_libraries(polygon_costmap_layer
  ${nav2_costmap_2d_LIBRARIES}
)

add_executable(polygon_publisher_node src/polygon_publisher_node.cpp)
target_link_libraries(polygon_publisher_node
  ${rclcpp_LIBRARIES}
  ${interactive_markers_LIBRARIES}
)

ament_target_dependencies(polygon_publisher_node
  "rclcpp"
  "interactive_markers"
  "geometry_msgs"
  "visualization_msgs"
)

install(
  DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS kimm_city_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  polygon_costmap_layer
  polygon_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
