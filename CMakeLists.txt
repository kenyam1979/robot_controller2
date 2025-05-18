cmake_minimum_required(VERSION 3.16)
project(robot_controller2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -pthread)
  add_compile_options(-g3 -O0)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/ken/ros2_ws/pigpio-master/util)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(pigpio REQUIRED)


## COMPILE
add_library(
  robot_controller2
  SHARED
  hardware/diffbot_system.cpp
  hardware/motor_pid.cpp
  hardware/motor.cpp
  hardware/motor_encoder.cpp
)
target_compile_features(robot_controller2 PUBLIC cxx_std_17)
target_include_directories(robot_controller2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robot_controller2>
)
ament_target_dependencies(
  robot_controller2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(robot_controller2 PUBLIC pigpiod_if2)
target_link_libraries(robot_controller2 PUBLIC rt)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_controller2.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/robot_controller2
)
install(
  DIRECTORY description/ros2_control
  DESTINATION share/robot_controller2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/robot_controller2
)
install(TARGETS robot_controller2
  EXPORT export_robot_controller2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
#   ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
# endif()

## EXPORTS
ament_export_targets(export_robot_controller2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
