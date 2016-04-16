cmake_minimum_required(VERSION 2.8.3)
project(state_space_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_space_example src/state_space_example.cpp)
target_link_libraries(state_space_example
  ${catkin_LIBRARIES}
)