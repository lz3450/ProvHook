cmake_minimum_required(VERSION 3.8)
project(myturtlesim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

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

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Color.msg
  msg/Pose.msg
  msg/Sensor.msg
  msg/Scale.msg
  srv/Kill.srv
  srv/SetPen.srv
  srv/Spawn.srv
  srv/TeleportAbsolute.srv
  srv/TeleportRelative.srv
  action/RotateAbsolute.action
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

qt5_wrap_cpp(turtlesim_node_MOCS
  include/myturtlesim/turtle_frame.h
)

add_executable(turtlesim_node
  src/myturtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
  ${turtlesim_node_MOCS}
)
ament_target_dependencies(turtlesim_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
)
target_link_libraries(turtlesim_node ${cpp_typesupport_target})
target_link_libraries(turtlesim_node Qt5::Widgets)

add_executable(turtle_tele_op_key src/turtle_tele_op_key.cpp)
ament_target_dependencies(turtle_tele_op_key
  rclcpp
  rclcpp_action
  geometry_msgs)
target_link_libraries(turtle_tele_op_key ${cpp_typesupport_target})

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(turtle_tf2_broadcaster ${cpp_typesupport_target})

add_executable(turtle_tf2_sensor src/turtle_tf2_sensor.cpp)
ament_target_dependencies(turtle_tf2_sensor
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(turtle_tf2_sensor ${cpp_typesupport_target})

add_executable(turtle2_controller src/turtle2_controller.cpp)
ament_target_dependencies(turtle2_controller
  rclcpp
  geometry_msgs
)
target_link_libraries(turtle2_controller ${cpp_typesupport_target})

add_executable(scale_publisher src/scale_publisher.cpp)
ament_target_dependencies(scale_publisher
  rclcpp
)
target_link_libraries(scale_publisher ${cpp_typesupport_target})

install(TARGETS
  turtlesim_node
  turtle_tele_op_key
  turtle_tf2_broadcaster
  turtle_tf2_sensor
  turtle2_controller
  scale_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
