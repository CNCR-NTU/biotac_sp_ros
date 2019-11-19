cmake_minimum_required(VERSION 2.8.3)
project(biotac_sp_ros)

catkin_python_setup()

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cheetah_lib std_msgs genmsg)


## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(DIRECTORY srv FILES AddTwoInts.srv BadTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  DEPENDS 
  CATKIN_DEPENDS roscpp cheetah_lib std_msgs
  INCLUDE_DIRS include
  LIBRARIES
)

## Build publisher
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(biotac_sp_ros src/biotac.c src/cheetah.c src/biotac_sp_ros.cpp)
target_link_libraries(biotac_sp_ros ${catkin_LIBRARIES} -ldl)
add_dependencies(biotac_sp_ros ${PROJECT_NAME}_gencpp)

install(TARGETS biotac_sp_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
