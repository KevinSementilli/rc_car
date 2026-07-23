cmake_minimum_required(VERSION 3.8)
project(redcat_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  xacro
  hardware_interface
  pluginlib
  controller_manager
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  slam_toolbox
  navigation2
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the shared library for hardware abstraction
# add_library(${PROJECT_NAME} SHARED
#   hardware/stepper_system_hardware.cpp
# )

# Set include directories
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# target_include_directories(${PROJECT_NAME} PUBLIC
#     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
#     $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# Link dependencies
# ament_target_dependencies(
#   ${PROJECT_NAME} PUBLIC
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# target_compile_definitions(${PROJECT_NAME} PRIVATE "ROBOT_HARDWARE_BUILDING_DLL")

# Export hardware interface as a plugin
# pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

# INSTALL
# install(
#   DIRECTORY hardware/include/
#   DESTINATION include/${PROJECT_NAME}
# )
install(
  DIRECTORY config urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)
# install(TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# Export targets and dependencies
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
