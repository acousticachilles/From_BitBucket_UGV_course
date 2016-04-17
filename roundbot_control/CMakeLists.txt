cmake_minimum_required(VERSION 2.8.3)
project(roundbot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wheel_speed_controller diff_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motor_sim src/MotorSim.cpp)
target_link_libraries(motor_sim
  ${catkin_LIBRARIES}
)

add_library(wheel_speed_controller src/WheelSpeedController.cpp)
target_link_libraries(wheel_speed_controller
  ${catkin_LIBRARIES}
  motor_sim
)

add_library(diff_controller src/DiffController.cpp)
target_link_libraries(diff_controller
  ${catkin_LIBRARIES}
  motor_sim
)

install(TARGETS wheel_speed_controller diff_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES roundbot_control_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

