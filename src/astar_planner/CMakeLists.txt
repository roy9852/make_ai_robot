cmake_minimum_required(VERSION 3.8)
project(astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Path Planner Node (src/astar.cpp)
add_executable(path_planner_node src/path_planner_node.cpp src/astar.cpp)
ament_target_dependencies(path_planner_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs
  visualization_msgs
)

# Simulator Node (src/simulator_node.cpp)
add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(simulator_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  path_planner_node
  simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

