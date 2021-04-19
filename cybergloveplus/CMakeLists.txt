cmake_minimum_required(VERSION 2.4.6)
project(cybergloveplus)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_planning_interface
  roscpp
  message_generation
  moveit_msgs
)
find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  Calibration.msg
)
add_service_files(
  FILES
  CheckSelfCollision.srv
)
generate_messages(
    DEPENDENCIES

)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   message_runtime
   moveit_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_executable(cybergloveraw src/cybergloveraw.cpp src/run_raw.cpp)
add_executable(cyberglove_calib_min_max src/cybergloveraw.cpp src/cyberglove_calib_min_max.cpp src/run_calib_min_max.cpp)
add_executable(cyberglove_control src/cybergloveraw.cpp src/cyberglove_calib_min_max.cpp src/cyberglove_control.cpp src/run_control.cpp)

add_dependencies(cybergloveraw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cyberglove_calib_min_max ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cyberglove_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cybergloveraw ${catkin_LIBRARIES})
target_link_libraries(cyberglove_calib_min_max ${catkin_LIBRARIES})
target_link_libraries(cyberglove_control ${catkin_LIBRARIES})
