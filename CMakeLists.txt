cmake_minimum_required(VERSION 3.8)
project(bt_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -- Generate custom action interface -----------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
)

# -- Install Python package ----------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# -- Install Python executables ------------------------------------------------
install(PROGRAMS
  bt_planner/simulator_node.py
  bt_planner/robot_node.py
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
