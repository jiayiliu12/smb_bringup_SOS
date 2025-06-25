cmake_minimum_required(VERSION 3.5)
project(smb_bringup)
# Use C++17
add_compile_options(-std=c++17)
# Find required packages
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install files
install(
 DIRECTORY
 launch
 config
 rviz
 DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(
 ${PROJECT_NAME}
)
install(PROGRAMS
 ${PROJECT_NAME}/challenge.py
 DESTINATION lib/smb_bringup/${PROJECT_NAME}
)
# Ament package
ament_package()