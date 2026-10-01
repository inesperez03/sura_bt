cmake_minimum_required(VERSION 3.8)
project(sura_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(sura_actions REQUIRED)
find_package(sura_safety REQUIRED)
find_package(sura_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)

set(SURA_BT_NODE_SOURCES
  src/fleet_tree.cpp
  src/controller_switch_nodes.cpp
  src/operation_mode_nodes.cpp
  src/camera_nodes.cpp
  src/command_nodes.cpp
  src/sura_actions_nodes.cpp
  src/register_nodes.cpp
)

add_executable(bt_runner
  ${SURA_BT_NODE_SOURCES}
  src/safety_ask_coordinator.cpp
  src/bt_runner.cpp
)

add_executable(export_bt_catalog
  ${SURA_BT_NODE_SOURCES}
  src/export_bt_catalog.cpp
)

target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(export_bt_catalog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(bt_runner PUBLIC cxx_std_17)
target_compile_features(export_bt_catalog PUBLIC cxx_std_17)

ament_target_dependencies(bt_runner
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  sensor_msgs
  diagnostic_msgs
  cv_bridge
  lifecycle_msgs
  ament_index_cpp
  behaviortree_cpp_v3
  controller_manager_msgs
  sura_actions
  sura_safety
  sura_msgs
)

ament_target_dependencies(export_bt_catalog
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  sensor_msgs
  diagnostic_msgs
  cv_bridge
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
  nlohmann_json::nlohmann_json
  PkgConfig::TINYXML2
)

target_link_libraries(export_bt_catalog
  nlohmann_json::nlohmann_json
  PkgConfig::TINYXML2
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/safety_ask_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
    bt_runner
    export_bt_catalog
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_area_diagnostics test/test_area_diagnostics.cpp src/command_nodes.cpp)
  target_include_directories(test_area_diagnostics PRIVATE include)
  target_compile_features(test_area_diagnostics PRIVATE cxx_std_17)
  ament_target_dependencies(test_area_diagnostics rclcpp sura_msgs sura_safety behaviortree_cpp_v3)

  ament_add_gtest(test_fleet_tree test/test_fleet_tree.cpp src/fleet_tree.cpp src/operation_mode_nodes.cpp)
  target_include_directories(test_fleet_tree PRIVATE include)
  target_compile_features(test_fleet_tree PRIVATE cxx_std_17)
  target_compile_definitions(test_fleet_tree PRIVATE SURA_BT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_target_dependencies(test_fleet_tree diagnostic_msgs behaviortree_cpp_v3 rclcpp std_msgs)
  target_link_libraries(test_fleet_tree PkgConfig::TINYXML2)

  ament_add_gtest(test_safety_ask_coordinator
    test/test_safety_ask_coordinator.cpp
    src/safety_ask_coordinator.cpp
    src/fleet_tree.cpp)
  target_include_directories(test_safety_ask_coordinator PRIVATE include)
  target_compile_features(test_safety_ask_coordinator PRIVATE cxx_std_17)
  ament_target_dependencies(test_safety_ask_coordinator
    rclcpp std_msgs std_srvs sura_safety diagnostic_msgs behaviortree_cpp_v3)
  target_link_libraries(test_safety_ask_coordinator
    nlohmann_json::nlohmann_json PkgConfig::TINYXML2)
endif()

ament_package()
