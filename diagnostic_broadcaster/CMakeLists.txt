cmake_minimum_required(VERSION 3.8)
project(diagnostic_broadcaster)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)

# === Parameter library ===
generate_parameter_library(
  diagnostic_parameters
  src/diagnostic_parameters.yaml
)

# === Controller library ===
set(SOURCES
  src/diagnostic_broadcaster.cpp
)
set(HEADERS
  include/${PROJECT_NAME}/diagnostic_broadcaster.hpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  realtime_tools
  diagnostic_msgs
  pluginlib
  hardware_interface
)
target_link_libraries(${PROJECT_NAME} PRIVATE diagnostic_parameters)

# === Optional standalone node ===
add_executable(diagnostic_node src/diagnostic_broadcaster.cpp)
target_link_libraries(diagnostic_node PRIVATE
  ${PROJECT_NAME}
  diagnostic_parameters
  rclcpp::rclcpp
)

# === Plugin registration ===
pluginlib_export_plugin_description_file(controller_interface diagnostic_broadcaster_plugin.xml)

# === Installation ===
install(TARGETS
    ${PROJECT_NAME}
    diagnostic_node
    diagnostic_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES diagnostic_broadcaster_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# === Tests ===
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gmock(test_diagnostic_broadcaster test/test_diagnostic_broadcaster.cpp)
  target_include_directories(test_diagnostic_broadcaster PRIVATE include)
  target_link_libraries(test_diagnostic_broadcaster
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
  )
  ament_target_dependencies(test_diagnostic_broadcaster
    rclcpp
    rclcpp_lifecycle
    controller_interface
    diagnostic_msgs
    controller_manager
    hardware_interface
  )
endif()

# === Package exports ===
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  diagnostic_msgs
  pluginlib
  controller_manager
  hardware_interface
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
