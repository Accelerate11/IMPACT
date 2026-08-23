cmake_minimum_required(VERSION 3.8)
project(xq_gz_assets)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/xq_gz_assets.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/xq_gz_assets.sh.in"
)

ament_package()
