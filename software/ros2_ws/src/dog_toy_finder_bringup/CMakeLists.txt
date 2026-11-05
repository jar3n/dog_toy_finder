cmake_minimum_required(VERSION 3.8)
project(dog_toy_finder_bringup)

find_package(ament_cmake REQUIRED)

# This package contains only launch files and YAML configuration, so there is
# nothing to compile. Install both directories into the package share folder.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
