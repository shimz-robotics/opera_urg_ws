cmake_minimum_required(VERSION 3.8)
project(opera_urg)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_box_filter.py
  scripts/line_center.py
  scripts/urg_calibrate.py
  scripts/urg_world_cloud.py
  scripts/urg_position_reset.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
