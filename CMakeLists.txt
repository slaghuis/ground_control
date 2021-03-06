cmake_minimum_required(VERSION 3.5)
project(ground_control)

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
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GpioRead.srv"
  "srv/GpioWrite.srv"
  "srv/Hilo.srv"
)

add_executable(publish_position_node
  src/flight_position_node.cpp
)
ament_target_dependencies(publish_position_node
  "rclcpp"
  "std_msgs"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
)
 
add_executable(ground_control_node
  src/ground_control_node.cpp
  src/powered_state.cpp
  src/armed_state.cpp
  src/flight_state.cpp
)
ament_target_dependencies(ground_control_node
  "rclcpp"
  "navigation_interfaces"
)
rosidl_target_interfaces(ground_control_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(ground_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 

add_executable(gpio_node
  src/gpio_node.cpp
)
ament_target_dependencies(gpio_node
  "rclcpp"
)
rosidl_target_interfaces(gpio_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS publish_position_node
                ground_control_node
                gpio_node
 DESTINATION lib/${PROJECT_NAME})
 
ament_package()
