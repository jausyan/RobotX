cmake_minimum_required(VERSION 3.10)
project(vision_krti)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
unset(OpenCV_DIR CACHE)
find_package(
  OpenCV REQUIRED COMPONENTS core aruco calib3d highgui imgproc videoio
  PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4
  NO_DEFAULT_PATH
)

add_executable(vision_krti src/vision_krti.cpp)
ament_target_dependencies(vision_krti rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(vision_krti ${OpenCV_LIBS})

add_executable(vision_tag src/vision_tag.cpp)
ament_target_dependencies(vision_tag rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(vision_tag ${OpenCV_LIBS})

add_executable(vision_artag src/vision_artag.cpp)
ament_target_dependencies(vision_artag rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(vision_artag ${OpenCV_LIBS})

install(TARGETS
  vision_krti
  vision_tag
  vision_artag
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
