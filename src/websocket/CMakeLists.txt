cmake_minimum_required(VERSION 3.8)
project(websocket)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(cJSON REQUIRED)

add_library(${PROJECT_NAME}_lib
  # src/ws_msgs_manage.cpp
  src/websocket.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  robot_msgs
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC ${cjson_INCLUDE_DIRS})
include_directories(/usr/local/nclude/cjson)  # 根据实际情况调整路径
target_link_libraries(${PROJECT_NAME}_lib cjson)  # 直接链接 json-c

add_executable(websocket_node src/websocket_node.cpp)
target_link_libraries(websocket_node ${PROJECT_NAME}_lib)

target_include_directories(websocket_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(websocket_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  websocket_node
  "rclcpp"
  "std_msgs"
  "robot_msgs"
  "cJSON"
)



install(TARGETS websocket_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

include_directories(
  include/${PROJECT_NAME}
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
