cmake_minimum_required(VERSION 3.10)
project(segmentation_node)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------ Dependencies ------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters)
find_package(OpenCV REQUIRED)

# Optional OpenMP
find_package(OpenMP)
set(HAVE_OPENMP FALSE)
if(OpenMP_CXX_FOUND)
  set(HAVE_OPENMP TRUE)
endif()

# ------------ circle_crop -------------
add_executable(circle_crop src/circle_crop.cpp)

ament_target_dependencies(circle_crop
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(circle_crop PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(circle_crop
  ${PCL_LIBRARIES}
)

target_compile_definitions(circle_crop PRIVATE
  ${PCL_DEFINITIONS}
)

# Optimize + OpenMP only AFTER the target exists
target_compile_options(circle_crop PRIVATE -O3 -march=native)
if(HAVE_OPENMP)
  target_link_libraries(circle_crop OpenMP::OpenMP_CXX)
endif()

# ----------- Other nodes -------------
add_executable(plane_segmentation_ransac src/plane_segmentation_ransac.cpp)
ament_target_dependencies(plane_segmentation_ransac rclcpp sensor_msgs pcl_conversions)
target_include_directories(plane_segmentation_ransac PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(plane_segmentation_ransac ${PCL_LIBRARIES})
target_compile_definitions(plane_segmentation_ransac PRIVATE ${PCL_DEFINITIONS})

add_executable(plane_segmentation_gng src/plane_segmentation_gng.cpp)
ament_target_dependencies(plane_segmentation_gng rclcpp sensor_msgs pcl_conversions)
target_include_directories(plane_segmentation_gng PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(plane_segmentation_gng ${PCL_LIBRARIES})
target_compile_definitions(plane_segmentation_gng PRIVATE ${PCL_DEFINITIONS})

add_executable(landing_circle src/landing_circle.cpp)
ament_target_dependencies(landing_circle rclcpp sensor_msgs pcl_conversions)
target_include_directories(landing_circle PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(landing_circle ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# ------------- Install ----------------
install(TARGETS
  circle_crop
  plane_segmentation_ransac
  plane_segmentation_gng
  landing_circle
  DESTINATION lib/${PROJECT_NAME}
)

# ------------- Tests ------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
