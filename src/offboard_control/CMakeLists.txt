cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(hover_only src/hover_only.cpp)
ament_target_dependencies(hover_only rclcpp px4_msgs)

install(
  DIRECTORY include 
  DESTINATION include
)

install(TARGETS 
  hover_only 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
