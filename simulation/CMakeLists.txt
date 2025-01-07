cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

# C++ 실행 파일 추가
add_executable(kimm_city_node src/kimm_city_node.cpp)
ament_target_dependencies(kimm_city_node
  rclcpp
  gazebo_ros
  gazebo_msgs
)

install(
  DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS kimm_city_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
