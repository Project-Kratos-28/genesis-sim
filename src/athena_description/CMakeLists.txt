cmake_minimum_required(VERSION 3.8)
project(athena_description)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
    launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES ground_noise.png
  DESTINATION share/${PROJECT_NAME}
)
install( PROGRAMS
  script/genesis_launcher.py
  script/Ackermann.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
