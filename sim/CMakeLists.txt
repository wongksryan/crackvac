cmake_minimum_required(VERSION 3.20)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(state_publisher src/state_publisher.cpp)

target_link_libraries(state_publisher PUBLIC
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_TARGETS}
)

#build swerve_controller library
add_library(swerve_controller src/swerve_controller.cpp)
target_include_directories(swerve_controller PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(swerve_controller PUBLIC
  rclcpp::rclcpp
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# Install library
install(
  TARGETS swerve_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
# Install the executable
install(
  TARGETS state_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
#install export targets
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
#install other resources
install(
  DIRECTORY configs launch urdf
  DESTINATION share/${PROJECT_NAME}
)
# Export old-style CMake variables
ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()