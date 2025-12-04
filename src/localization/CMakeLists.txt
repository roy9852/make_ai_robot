cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##### CHANGE START #####

###### find dependencies ######
find_package(ament_cmake REQUIRED)

##### Copy folders to install directory #####
install(
  DIRECTORY 
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python entry points as executables
install(
  PROGRAMS
    localization/odom_localizer_node.py
    localization/global_localizer_node.py
    localization/utils.py
  DESTINATION lib/${PROJECT_NAME}
)

##### CHANGE END #####

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