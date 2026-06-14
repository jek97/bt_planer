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
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

# -- Generate the custom Move action ------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
)

# -- Simulator executable ------------------------------------------------------
add_executable(simulator src/simulator.cpp)

ament_target_dependencies(simulator
  rclcpp
  rclcpp_action
  OpenCV
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(simulator "${cpp_typesupport_target}" ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

target_include_directories(simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(simulator PUBLIC c_std_99 cxx_std_17)

install(TARGETS simulator
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