cmake_minimum_required(VERSION 3.8)
project(path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Suppress warnings from legacy code structure
  add_compile_options(
    -Wno-reorder                       # Suppress member initialization order warnings
    -Wno-sign-compare                  # Suppress signed/unsigned comparison warnings
    -Wno-unused-parameter              # Suppress unused parameter warnings
    -Wno-deprecated-copy               # Suppress deprecated copy constructor warnings
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})



add_executable(path_tracker_node
  src/path_tracker.cpp
  src/action.cpp
  src/communication.cpp
  src/cost.cpp
  src/dummy_path.cpp
  src/dynamics.cpp
  src/misc.cpp
  src/regression.cpp
  src/state.cpp
  src/transform.cpp
  src/visualization.cpp
)

ament_target_dependencies(path_tracker_node
  rclcpp
  ament_index_cpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(path_tracker_node
  ${EIGEN3_LIBRARIES}
)

install(TARGETS
  path_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  path_tracker
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    path_tracker/move_robot.py
  DESTINATION lib/${PROJECT_NAME}       
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
