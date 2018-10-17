cmake_minimum_required(VERSION 2.4.6)
project(cybergloveplusui)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cybergloveplus
)

include_directories(${catkin_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs cybergloveplus
)

qt5_add_resources(QT_RESOURCES_CPP 
  resources/images.qrc
)

qt5_wrap_cpp(QT_MOC_HPP 
  include/cybergloveplusui/formCybergloveCalibration.hpp 
  include/cybergloveplusui/qnode.hpp
)

qt5_wrap_ui(QT_FORMS_HPP 
  ui/cyberglove_calib.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(cybergloveplusui 
  src/formCybergloveCalibration.cpp 
  src/main.cpp 
  src/qnode.cpp 
  ${QT_RESOURCES_CPP} 
  ${QT_FORMS_HPP} 
  ${QT_MOC_HPP}
)

target_link_libraries(cybergloveplusui ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(cybergloveplusui ${catkin_EXPORTED_TARGETS})
