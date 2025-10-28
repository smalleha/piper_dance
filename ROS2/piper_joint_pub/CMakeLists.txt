cmake_minimum_required(VERSION 3.8)
project(piper_joint_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(piper_joint_pub src/piper_joint_pub.cpp)
ament_target_dependencies(piper_joint_pub rclcpp sensor_msgs)
target_link_libraries(piper_joint_pub yaml-cpp)
install(TARGETS piper_joint_pub DESTINATION lib/${PROJECT_NAME})

add_executable(piper_joint_yaml src/piper_joint_yaml.cpp)
ament_target_dependencies(piper_joint_yaml rclcpp sensor_msgs)
target_link_libraries(piper_joint_yaml yaml-cpp)
install(TARGETS piper_joint_yaml DESTINATION lib/${PROJECT_NAME})

add_executable(piper_multi_joint src/piper_multi_joint.cpp)
ament_target_dependencies(piper_multi_joint rclcpp sensor_msgs)
target_link_libraries(piper_multi_joint yaml-cpp)
install(TARGETS piper_multi_joint DESTINATION lib/${PROJECT_NAME})

add_executable(piper_joint_pub_eighteen src/piper_joint_pub_eighteen.cpp)
ament_target_dependencies(piper_joint_pub_eighteen rclcpp sensor_msgs)
target_link_libraries(piper_joint_pub_eighteen yaml-cpp)
install(TARGETS piper_joint_pub_eighteen DESTINATION lib/${PROJECT_NAME})

add_executable(piper_joint_pub_tenth src/piper_joint_pub_tenth.cpp)
ament_target_dependencies(piper_joint_pub_tenth rclcpp sensor_msgs)
target_link_libraries(piper_joint_pub_tenth yaml-cpp)
install(TARGETS piper_joint_pub_tenth DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)


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
