cmake_minimum_required(VERSION 3.5)
project(bumperbot_description)


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch meshes models urdf rviz  photos worlds
  DESTINATION share/${PROJECT_NAME}
)
# install(DIRECTORY worlds/
#         DESTINATION share/${PROJECT_NAME}/worlds)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()