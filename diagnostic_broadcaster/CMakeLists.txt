cmake_minimum_required(VERSION 3.8)
project(diagnostic_broadcaster)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp 
  rclcpp_lifecycle
  realtime_tools 
  controller_interface
  diagnostic_msgs
  hardware_interface
  generate_parameter_library
  parameter_traits
  realtime_tools
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  diagnostic_parameters
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# === Plugin registration ===
pluginlib_export_plugin_description_file(controller_interface diagnostic_broadcaster_plugin.xml)

# === Installation ===
install(TARGETS
    ${PROJECT_NAME}
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
# install(FILES diagnostic_broadcaster_plugin.xml
#   DESTINATION share/${PROJECT_NAME}
# )

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
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()