cmake_minimum_required(VERSION 3.8)
project(safety_envelope)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(
  headers
)

# Create the safety envelope library
add_library(${PROJECT_NAME}
  safety_envelope.cpp
  microcontroller_stub.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/headers>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Create executable for the safety envelope node
add_executable(safety_envelope_node
  safety_envelope.cpp
)

target_include_directories(safety_envelope_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/headers>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(safety_envelope_node
  rclcpp
  geometry_msgs
)

# Install header files
install(
  FILES headers/safety_envelope.hpp
  DESTINATION include/${PROJECT_NAME}
)

# Install documentation
install(
  FILES safety_envelope_notes.md
  DESTINATION share/${PROJECT_NAME}/doc
)

# Install executables
install(
  TARGETS safety_envelope_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
