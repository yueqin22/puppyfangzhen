cmake_minimum_required(VERSION 3.8)
project(puppy_description)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes config rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
