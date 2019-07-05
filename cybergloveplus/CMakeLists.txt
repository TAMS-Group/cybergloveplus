cmake_minimum_required(VERSION 2.4.6)
project(cybergloveplus)


find_package(catkin REQUIRED COMPONENTS
  # eigen
  roscpp
  message_generation

)

add_message_files(
  FILES
  Calibration.msg
)

generate_messages(
    DEPENDENCIES

)

catkin_package(
   CATKIN_DEPENDS roscpp  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(cybergloveraw src/cybergloveraw.cpp src/run_raw.cpp)
add_executable(cyberglove_calib_min_max src/cybergloveraw.cpp src/cyberglove_calib_min_max.cpp src/run_calib_min_max.cpp)
add_executable(cyberglove_control src/cybergloveraw.cpp src/cyberglove_calib_min_max.cpp src/cyberglove_control.cpp src/run_control.cpp)

add_dependencies(cybergloveraw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cyberglove_calib_min_max ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cyberglove_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cybergloveraw ${catkin_LIBRARIES})
target_link_libraries(cyberglove_calib_min_max ${catkin_LIBRARIES})
target_link_libraries(cyberglove_control ${catkin_LIBRARIES})
