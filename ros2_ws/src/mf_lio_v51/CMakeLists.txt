cmake_minimum_required(VERSION 3.8)
project(mf_lio_v51)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# AVX-512 alignment fix: Eigen fixed-size matrices on std::thread stacks
# require at most 16-byte (SSE2) alignment to avoid SIGSEGV with march=native
# on AVX-512-capable CPUs where the compiler would otherwise emit 64-byte
# aligned loads that fault on non-64-byte-aligned thread stacks.
add_compile_options(-O3 -march=native
    -DEIGEN_MAX_ALIGN_BYTES=16
    -DEIGEN_MAX_STATIC_ALIGN_BYTES=16)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)

# Third-party
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  include/mf_lio
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(mf_lio_v51_node
  src/mf_lio_node.cpp
  src/mf_lio_estimator.cpp
  src/imu_processor.cpp
  src/sliding_window.cpp
  include/mf_lio/ikd-Tree/ikd_Tree.cpp
)

target_link_libraries(mf_lio_v51_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(mf_lio_v51_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  livox_ros_driver2
  pcl_conversions
)

# Install
install(TARGETS mf_lio_v51_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
