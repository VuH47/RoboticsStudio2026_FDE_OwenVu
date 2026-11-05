cmake_minimum_required(VERSION 3.8)
project(fde_gui)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/ui_bridge_node.py
    scripts/operator_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY operator_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install GUI images/icons, including all subfolders
install(
  DIRECTORY assets
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()