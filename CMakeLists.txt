cmake_minimum_required(VERSION 3.8)
project(spacenav-test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yarp_control_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)

add_executable(spacenav-test src/main.cpp)
ament_target_dependencies(spacenav-test rclcpp geometry_msgs sensor_msgs yarp_control_msgs orocos_kdl)
target_link_libraries(spacenav-test ${orocos_kdl_LIBRARIES})
target_include_directories(spacenav-test PUBLIC ${orocos_kdl_INCLUDE_DIRS})


install(TARGETS
  spacenav-test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
