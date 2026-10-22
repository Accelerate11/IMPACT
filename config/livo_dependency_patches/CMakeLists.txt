cmake_minimum_required(VERSION 3.16)
project(impact_livo_dependency_wrapper VERSION 1.0.0 LANGUAGES CXX)

# A build-system wrapper for pinned Robotic-Developer-Road/rpg_vikit sources.
# Upstream sources and their LICENSE remain in the isolated dependency clone.
# Numerical camera models stay upstream. The explicit patch fixes bounded
# ROS2 parameter-service discovery without a shell-command fallback.
if(NOT IS_DIRECTORY "${VIKIT_SOURCE_DIR}/vikit_common")
  message(FATAL_ERROR "VIKIT_SOURCE_DIR must point to the pinned rpg_vikit clone")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus 1.22.10 EXACT CONFIG REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)
foreach(dependency rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
  find_package(${dependency} REQUIRED)
endforeach()

set(common_dir "${VIKIT_SOURCE_DIR}/vikit_common")
set(common_sources atan_camera.cpp omni_camera.cpp math_utils.cpp vision.cpp
  performance_monitor.cpp robust_cost.cpp user_input_thread.cpp pinhole_camera.cpp
  equidistant_camera.cpp polynomial_camera.cpp homography.cpp img_align.cpp)
list(TRANSFORM common_sources PREPEND "${common_dir}/src/")
add_library(vikit_common SHARED ${common_sources})
add_library(vikit_common::vikit_common ALIAS vikit_common)
target_include_directories(vikit_common PUBLIC
  $<BUILD_INTERFACE:${common_dir}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(vikit_common PUBLIC Sophus::Sophus Eigen3::Eigen fmt::fmt
  Threads::Threads ${OpenCV_LIBRARIES})
target_include_directories(vikit_common SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_options(vikit_common PRIVATE -Wl,--no-undefined)

set(ros_dir "${VIKIT_SOURCE_DIR}/vikit_ros")
add_library(vikit_ros SHARED "${ros_dir}/src/output_helper.cpp" "${ros_dir}/src/camera_loader.cpp")
add_library(vikit_ros::vikit_ros ALIAS vikit_ros)
target_include_directories(vikit_ros PUBLIC
  $<BUILD_INTERFACE:${ros_dir}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vikit_ros rclcpp geometry_msgs nav_msgs sensor_msgs
  visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(vikit_ros vikit_common)
target_link_options(vikit_ros PRIVATE -Wl,--no-undefined)

install(TARGETS vikit_common EXPORT vikitCommonTargets LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib INCLUDES DESTINATION include)
install(TARGETS vikit_ros EXPORT vikitRosTargets LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib INCLUDES DESTINATION include)
install(DIRECTORY "${common_dir}/include/" "${ros_dir}/include/" DESTINATION include)
install(EXPORT vikitCommonTargets FILE vikitCommonTargets.cmake
  NAMESPACE vikit_common:: DESTINATION share/vikit_common/cmake)
install(EXPORT vikitRosTargets FILE vikitRosTargets.cmake
  NAMESPACE vikit_ros:: DESTINATION share/vikit_ros/cmake)
foreach(package vikit_common vikit_ros)
  configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/${package}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${package}Config.cmake"
    INSTALL_DESTINATION "share/${package}/cmake")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${package}Config.cmake"
    DESTINATION "share/${package}/cmake")
endforeach()
file(GLOB upstream_licenses "${VIKIT_SOURCE_DIR}/LICENSE*" "${common_dir}/LICENSE*" "${ros_dir}/LICENSE*")
if(upstream_licenses)
  install(FILES ${upstream_licenses} DESTINATION share/impact_livo_dependencies/licenses)
endif()
