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
  include
)

# Create the safety envelope library
add_library(${PROJECT_NAME}
  src/safety_envelope.cpp
  src/microcontroller_stub.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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
  src/safety_envelope_node.cpp
)

target_include_directories(safety_envelope_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(safety_envelope_node
  rclcpp
  geometry_msgs
)

target_link_libraries(safety_envelope_node
  ${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install documentation
install(
  DIRECTORY doc/
  DESTINATION share/${PROJECT_NAME}/doc
)

# Install configuration files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install executables
install(
  TARGETS safety_envelope_node
  DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_envelope
    test/test_safety_envelope.cpp
  )
  target_include_directories(test_safety_envelope PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_safety_envelope
    rclcpp
    geometry_msgs
  )
  target_link_libraries(test_safety_envelope
    ${PROJECT_NAME}
  )
endif()

# Export package information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
