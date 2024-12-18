cmake_minimum_required(VERSION 3.8)
project(bumperbot_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the interfaces (services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "srv/GetTransform.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
