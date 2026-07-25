cmake_minimum_required(VERSION 3.8)
project(puppy_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/coppelia_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coppelia_bridge.py
)

install(PROGRAMS scripts/odom_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_tf_publisher.py
)

install(PROGRAMS scripts/clock_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME clock_publisher.py
)

ament_package()
