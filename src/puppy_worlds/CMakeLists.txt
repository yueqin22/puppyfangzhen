cmake_minimum_required(VERSION 3.8)
project(puppy_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/clock_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
