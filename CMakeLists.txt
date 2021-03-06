cmake_minimum_required(VERSION 2.8.3)
project(ardrone_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ROI.msg
  ROINumber.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(image_process
  src/image_process.cpp 
)


add_executable(image_control 
  src/image_control.cpp
)
target_link_libraries(image_control
  image_process 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(cvtrials 
  src/cvtrials.cpp
)
target_link_libraries(cvtrials
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/keyboard_control.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

