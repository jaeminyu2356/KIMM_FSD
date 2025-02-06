cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_2dmap)

# C++ 표준 설정 (원하는 버전으로 조정 가능)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 필수 라이브러리 찾기
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

###########
## Build ##
###########

# include 디렉토리 설정
# - include : 프로젝트 내 헤더 폴더
# - ${OpenCV_INCLUDE_DIRS} : OpenCV 헤더 경로
# - ${PCL_INCLUDE_DIRS} : PCL 헤더 경로
# - ${Boost_INCLUDE_DIRS} : Boost 헤더 경로
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 실행 파일 생성
add_executable(pointcloud_to_2dmap src/pointcloud_to_2dmap.cpp)

# 라이브러리 링크
target_link_libraries(pointcloud_to_2dmap
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

