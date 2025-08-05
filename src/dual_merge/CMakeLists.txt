cmake_minimum_required(VERSION 3.10)
project(dual_merge)

# Find required packages
find_package(ament_cmake REQUIRED)

# Install resource folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()