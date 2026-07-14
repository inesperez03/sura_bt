cmake_minimum_required(VERSION 3.8)
project(sura_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sura_safety REQUIRED)

add_executable(bt_runner
  src/bt_runner.cpp
)

target_compile_features(bt_runner PUBLIC cxx_std_14)

ament_target_dependencies(bt_runner
  rclcpp
  ament_index_cpp
  behaviortree_cpp_v3
  sura_safety
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
