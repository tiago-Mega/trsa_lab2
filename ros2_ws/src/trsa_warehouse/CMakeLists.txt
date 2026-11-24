cmake_minimum_required(VERSION 3.8)
project(trsa_warehouse)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config assets DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check scripts/validate_model
        scripts/sensor_adapter scripts/wheel_odom_interface scripts/nav_check scripts/fusion_record scripts/validate_navigation
        DESTINATION lib/${PROJECT_NAME})
ament_package()
