cmake_minimum_required(VERSION 3.8)
project(segmentation_node)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)   # provide Eigen3::Eigen

# PCL definitions/includes
add_definitions(${PCL_DEFINITIONS})

# ---- cylinder_crop ----
add_executable(cylinder_crop src/cylinder_crop.cpp)
ament_target_dependencies(cylinder_crop rclcpp sensor_msgs pcl_conversions)
target_include_directories(cylinder_crop PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cylinder_crop ${PCL_LIBRARIES})
target_compile_definitions(cylinder_crop PRIVATE ${PCL_DEFINITIONS})

# ---- plane_segmentation_gng ----
add_executable(plane_segmentation_gng src/plane_segmentation_gng.cpp)
ament_target_dependencies(plane_segmentation_gng rclcpp sensor_msgs pcl_conversions)
target_include_directories(plane_segmentation_gng PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(plane_segmentation_gng ${PCL_LIBRARIES})
target_compile_definitions(plane_segmentation_gng PRIVATE ${PCL_DEFINITIONS})

# ---- plane_segmentation_ransac ----
add_executable(plane_segmentation_ransac src/plane_segmentation_ransac.cpp)
ament_target_dependencies(plane_segmentation_ransac rclcpp sensor_msgs pcl_conversions)
target_include_directories(plane_segmentation_ransac PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(plane_segmentation_ransac ${PCL_LIBRARIES})
target_compile_definitions(plane_segmentation_ransac PRIVATE ${PCL_DEFINITIONS})

# ---- landing_circle (pakai TF & Eigen) ----
add_executable(landing_circle src/landing_circle.cpp)
ament_target_dependencies(landing_circle
  rclcpp sensor_msgs pcl_conversions
  tf2 tf2_ros tf2_eigen
)
target_include_directories(landing_circle PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(landing_circle ${PCL_LIBRARIES} Eigen3::Eigen)
target_compile_definitions(landing_circle PRIVATE ${PCL_DEFINITIONS})

# ---- install headers (opsional) ----
install(DIRECTORY include DESTINATION include)

# ---- install binaries ----
install(TARGETS
  landing_circle
  cylinder_crop
  plane_segmentation_gng
  plane_segmentation_ransac
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# ---- lint (opsional) ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
