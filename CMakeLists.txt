cmake_minimum_required(VERSION 3.0.2)
project(sparse_slam)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sparse_slam
#  CATKIN_DEPENDS cv_bridge roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(self_library src/system.cpp src/track.cpp src/extractor.cpp)
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}  self_library ${catkin_LIBRARIES})