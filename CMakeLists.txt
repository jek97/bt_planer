cmake_minimum_required(VERSION 3.8)
project(bt_planner)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -- Generate custom action interface -----------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
)

# -- Install Python scripts as executables ------------------------------------
install(PROGRAMS
  scripts/simulator_node.py
  scripts/robot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# -- Install data directories --------------------------------------------------
install(DIRECTORY environment/
  DESTINATION share/${PROJECT_NAME}/environment
)

install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
