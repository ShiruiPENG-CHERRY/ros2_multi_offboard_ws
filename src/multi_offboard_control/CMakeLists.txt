cmake_minimum_required(VERSION 3.8)
project(multi_offboard_control)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
 
add_executable(multi_offboard_control src/multi_offboard_control.cpp)
ament_target_dependencies(multi_offboard_control rclcpp px4_msgs)
 
install(TARGETS
  multi_offboard_control
  DESTINATION lib/${PROJECT_NAME}
)
 
ament_package()
 