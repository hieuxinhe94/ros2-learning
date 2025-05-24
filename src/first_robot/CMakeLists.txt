cmake_minimum_required(VERSION 3.5)
project(first_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# main program
install(
  DIRECTORY config description launch mesh worlds ai_models inteligent
  DESTINATION share/${PROJECT_NAME}
)

# actionable program
install( 
  DIRECTORY scripts move ai_models inteligent
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/twist_to_twiststamped.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  move/random_mover.py move/patrol_mover.py  inteligent/obstacle_avoid_node_by_camera.py inteligent/obstacle_avoid_node_by_depth_camera.py  inteligent/semantic_explorer.py  inteligent/action_scheduler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
