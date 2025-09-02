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
find_package(visualization_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

add_executable(plane_segmentation_temp src/plane_segmentation_temp.cpp)
ament_target_dependencies(plane_segmentation_temp rclcpp sensor_msgs pcl_conversions)
target_link_libraries(plane_segmentation_temp ${PCL_LIBRARIES})

add_executable(plane_segmentation src/plane_segmentation.cpp)
ament_target_dependencies(plane_segmentation rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(plane_segmentation ${PCL_LIBRARIES})

add_executable(clustering src/clustering.cpp)
ament_target_dependencies(clustering rclcpp sensor_msgs pcl_conversions)
target_link_libraries(clustering ${PCL_LIBRARIES})

add_executable(landing_circle src/landing_circle.cpp)
ament_target_dependencies(landing_circle rclcpp visualization_msgs pcl_conversions sensor_msgs)

install(TARGETS
  zed_segmentation
  landing_circle
  cloud_save
  plane_segmentation
  clustering
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
