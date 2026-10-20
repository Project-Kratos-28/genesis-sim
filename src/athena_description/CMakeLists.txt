cmake_minimum_required(VERSION 3.8)
project(athena_description)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Assets (URDF, meshes, launch files, ...)
install(
  DIRECTORY
    meshes
    urdf
    launch
    config
    img
  DESTINATION share/${PROJECT_NAME}
)

# Executable script
install(PROGRAMS
  script/genesis_launcher.py
  script/check.py
  DESTINATION lib/${PROJECT_NAME}
)

# Python packages the launcher imports; must sit next to it in lib/
install(
  DIRECTORY
    script/genesis_bridge
    script/steering
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()