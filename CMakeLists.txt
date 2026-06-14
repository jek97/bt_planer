cmake_minimum_required(VERSION 3.8)
project(bt_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# -- Generate the custom Move action ------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
)

# -- Simulator executable ------------------------------------------------------
add_executable(simulator src/simulator.cpp)

ament_target_dependencies(simulator
  rclcpp
  rclcpp_action
  ament_index_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(simulator
  "${cpp_typesupport_target}"
  nlohmann_json::nlohmann_json
  sfml-graphics
  sfml-window
  sfml-system
)

target_include_directories(simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(simulator PUBLIC c_std_99 cxx_std_17)

add_executable(robot src/robot.cpp)

ament_target_dependencies(robot
  rclcpp
  rclcpp_action
)

rosidl_get_typesupport_target(cpp_typesupport_target_robot ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot "${cpp_typesupport_target_robot}")

target_compile_features(robot PUBLIC cxx_std_17)

install(TARGETS simulator robot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY environment/
  DESTINATION share/${PROJECT_NAME}/environment
)

# -- Tests ---------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()