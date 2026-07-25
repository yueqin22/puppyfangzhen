cmake_minimum_required(VERSION 3.8)
project(puppy_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install Python scripts
install(PROGRAMS
  scripts/patrol.py
  scripts/simple_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
