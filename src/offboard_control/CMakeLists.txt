cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(hover_only src/hover_only.cpp)
target_include_directories(hover_only PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hover_only
  rclcpp
  px4_msgs
)

add_executable(drone_kinematics src/drone_kinematics.cpp)
target_include_directories(drone_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drone_kinematics
  rclcpp
  px4_msgs
  geometry_msgs
  tf2_ros
  builtin_interfaces
)

add_executable(zed_vio_to_px4_second src/zed_vio_to_px4_second.cpp)
target_include_directories(zed_vio_to_px4_second PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(zed_vio_to_px4_second
  rclcpp
  nav_msgs
  px4_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  builtin_interfaces
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  hover_only
  drone_kinematics
  zed_vio_to_px4_second
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
