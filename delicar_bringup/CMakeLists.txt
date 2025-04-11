cmake_minimum_required(VERSION 3.8)
project(delicar_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  hardware/include
  ${MODBUS_INCLUDE_DIRS}
)

## COMPILE
add_library(
  delicar_bringup
  SHARED
  hardware/carlikebot_system.cpp
)
target_compile_features(delicar_bringup PUBLIC cxx_std_17)
target_include_directories(delicar_bringup PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/delicar_bringup>
)
ament_target_dependencies(
  delicar_bringup PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(delicar_bringup PUBLIC ${MODBUS_LIBRARIES})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface delicar_controller.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/delicar_bringup
)

install(
  DIRECTORY launch config
  DESTINATION share/delicar_bringup
)

install(TARGETS delicar_bringup
  EXPORT export_delicar_bringup
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_delicar_bringup HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
