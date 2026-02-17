cmake_minimum_required(VERSION 3.8)
project(urc_arm_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(
  DIRECTORY meshes urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/keyboard_arm_teleop.py
    src/keyboard_cartesian_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()