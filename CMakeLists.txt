cmake_minimum_required(VERSION 3.8)
project(sura_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(sura_actions REQUIRED)
find_package(sura_safety REQUIRED)
find_package(sura_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(bt_runner
  src/controller_switch_nodes.cpp
  src/operation_mode_nodes.cpp
  src/send_wrench_node.cpp
  src/sura_actions_nodes.cpp
  src/bt_runner.cpp
)

target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(bt_runner PUBLIC cxx_std_17)

ament_target_dependencies(bt_runner
  rclcpp
  rclcpp_action
  std_msgs
  lifecycle_msgs
  ament_index_cpp
  behaviortree_cpp_v3
  controller_manager_msgs
  sura_actions
  sura_safety
  sura_msgs
)

target_link_libraries(bt_runner
  yaml-cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
