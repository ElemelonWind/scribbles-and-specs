cmake_minimum_required(VERSION 3.5)
project(specs)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  apriltag_test.py
  localization.py
  video_display.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
