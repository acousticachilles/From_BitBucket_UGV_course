cmake_minimum_required(VERSION 2.8.3)
project(customer_generators)

set(${PROJECT_NAME}_CATKIN_DEPS
  costmap_2d
  navfn
  base_local_planner
  roscpp
  tf
  visualization_msgs
  ugv_course_libs
  dynamic_reconfigure
  interactive_markers
)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(customer_generator src/customer_generator.cpp)
target_link_libraries(customer_generator 
  ${catkin_LIBRARIES}
)

