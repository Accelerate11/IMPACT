cmake_minimum_required(VERSION 3.8)
project(xq_gz_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(xq_gz_bridge_conversions
  src/conversions.cpp
)
target_include_directories(xq_gz_bridge_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xq_gz_bridge_conversions
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
)
target_link_libraries(xq_gz_bridge_conversions
  gz-msgs10::gz-msgs10
)

add_executable(xq_gz_bridge_node
  src/xq_gz_bridge_node.cpp
)
ament_target_dependencies(xq_gz_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
)
target_link_libraries(xq_gz_bridge_node
  xq_gz_bridge_conversions
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(
  TARGETS
    xq_gz_bridge_conversions
    xq_gz_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  if(TARGET test_conversions)
    ament_target_dependencies(test_conversions
      builtin_interfaces
      geometry_msgs
      nav_msgs
      sensor_msgs
    )
    target_link_libraries(test_conversions
      xq_gz_bridge_conversions
      gz-msgs10::gz-msgs10
    )
  endif()

  add_test(
    NAME test_shutdown_race
    COMMAND bash
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_shutdown_race.sh
      $<TARGET_FILE:xq_gz_bridge_node>
  )
  set_tests_properties(test_shutdown_race PROPERTIES TIMEOUT 45)
endif()

ament_export_include_directories(include)
ament_package()
