cmake_minimum_required(VERSION 3.5)
project(graph_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/world_gen.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
