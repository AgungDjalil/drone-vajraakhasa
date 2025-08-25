cmake_minimum_required(VERSION 3.8)
project(segmentation_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(zed_segmentation src/zed_segmentation.cpp)
ament_target_dependencies(zed_segmentation   
  rclcpp 
  sensor_msgs 
  pcl_conversions 
  pcl_msgs 
  rclcpp 
  sensor_msgs
)
target_link_libraries(zed_segmentation ${PCL_LIBRARIES})

add_executable(cloud_save src/cloud_save.cpp)
ament_target_dependencies(cloud_save rclcpp sensor_msgs pcl_conversions)
target_link_libraries(cloud_save ${PCL_LIBRARIES})

install(TARGETS
  zed_segmentation
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  cloud_save
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
