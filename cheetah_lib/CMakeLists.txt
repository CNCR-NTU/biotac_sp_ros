cmake_minimum_required(VERSION 2.8.3)
project(cheetah_lib)

message("Cheetah version v3.10")
message("Last Update: 31/01/2019")
message("Author: Pedro Machado pedro.baptistamachado@ntu.ac.uk")

configure_file(cheetah.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/cheetah.so COPYONLY)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES cheetah
)


