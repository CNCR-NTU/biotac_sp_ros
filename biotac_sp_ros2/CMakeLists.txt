cmake_minimum_required(VERSION 3.16)
project(biotac_sp_ros2)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message and service files
set(msg_files
  "msg/Floats.msg"
  "msg/HeaderString.msg"
)

set(srv_files
  "srv/AddTwoInts.srv"
  "srv/BadTwoInts.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Create C++ executable
add_executable(biotac_sp_ros2_node
  src/biotac_sp_ros2.cpp
  src/biotac.c
  src/cheetah.c
)

# Include headers
target_include_directories(biotac_sp_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(biotac_sp_ros2_node
  rclcpp
  std_msgs
)

# Link against generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(biotac_sp_ros2_node "${cpp_typesupport_target}")

# Install targets and headers
install(TARGETS
  biotac_sp_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python launch script
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python visualiser node into libexec dir
install(PROGRAMS
  src/biotac_sp_ros2/visualise_biotac.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visualise_biotac
)

# ====== NEW: Install cheetah.so shared library ======
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cheetah.so
  ${CMAKE_CURRENT_BINARY_DIR}/cheetah.so
  COPYONLY
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cheetah.so
  DESTINATION lib
)

# Export interfaces
ament_export_dependencies(rosidl_default_runtime)
ament_package()
