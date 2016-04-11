cmake_minimum_required(VERSION 2.8.3)
project(final_project)

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

generate_dynamic_reconfigure_options(
  cfg/customer_type.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner
  ${catkin_LIBRARIES}
)


add_executable(customer src/customer.cpp)
add_dependencies(customer ${PROJECT_NAME}_gencfg)
target_link_libraries(customer
  ${catkin_LIBRARIES}
)


add_executable(way_point_manager src/way_point_manager.cpp)
target_link_libraries(way_point_manager
  ${catkin_LIBRARIES}
)