cmake_minimum_required(VERSION 3.8)
project(bumperbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)


# Install the Python package
ament_python_install_package(${PROJECT_NAME})
# Install standalone Python executables
add_executable(kalman_filter src/kalman_filter.cpp)
target_include_directories(kalman_filter PRIVATE ${PROJECT_SOURCE_DIR}/include)
ament_target_dependencies(kalman_filter rclcpp nav_msgs sensor_msgs)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
kalman_filter
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/kalman_filter.py
  ${PROJECT_NAME}/kalman_filter_pos.py
  ${PROJECT_NAME}/imu_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)




ament_package()