cmake_minimum_required(VERSION 3.8)
project(b2_manual_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(robot_msgs REQUIRED)

add_executable(b2_sport_ctrl_node src/b2_sport_ctrl_node.cpp)
add_executable(motor_ctrl_2_twist src/motor_ctrl_2_twist.cpp)
target_include_directories(b2_sport_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(b2_sport_ctrl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  b2_sport_ctrl_node
  "rclcpp"
  "geometry_msgs"
  "unitree_api"
  "robot_msgs"
)

ament_target_dependencies(
  motor_ctrl_2_twist
  "rclcpp"
  "geometry_msgs"
  "unitree_api"
  "robot_msgs"
)

install(TARGETS b2_sport_ctrl_node motor_ctrl_2_twist
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
