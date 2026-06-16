cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

set(projectLibSources
  src/testing/drone_controller.cpp
  utils/control_.cpp
  utils/math_.cpp
  utils/geo_.cpp
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(control src/mission/control.cpp ${projectLibSources})
ament_target_dependencies(control rclcpp std_msgs geometry_msgs sensor_msgs mavros_msgs std_srvs)

add_executable(testing src/mission/testing.cpp ${projectLibSources})
ament_target_dependencies(testing rclcpp std_msgs geometry_msgs sensor_msgs mavros_msgs std_srvs)

install(TARGETS
  control
  testing
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
