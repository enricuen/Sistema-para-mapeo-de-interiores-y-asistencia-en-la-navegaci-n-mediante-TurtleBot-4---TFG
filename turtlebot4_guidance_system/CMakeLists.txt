cmake_minimum_required(VERSION 3.5)
project(turtlebot4_guidance_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  maps
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/robot_guia_gui.py
  scripts/pruebasnav.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
