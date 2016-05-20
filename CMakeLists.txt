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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(image_process
  src/image_process.cpp 
)

add_executable(position_control 
  src/position_control.cpp
)
target_link_libraries(position_control 
  ${catkin_LIBRARIES}
)

add_executable(mission 
  src/mission_test.cpp
)
target_link_libraries(mission 
  ${catkin_LIBRARIES}
)

add_executable(image_control 
  src/image_control.cpp
)
target_link_libraries(image_control
  image_process 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(find_contour 
  src/find_contour.cpp
)
target_link_libraries(find_contour
  image_process 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(serial_control 
  src/serial_control.cpp
)
target_link_libraries(serial_control
  ${catkin_LIBRARIES}
)




## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/keyboard_control.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

