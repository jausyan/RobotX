cmake_minimum_required(VERSION 3.10)
project(vision_geo)

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
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(vision_geo
  src/vision_geo.cpp
  utils/vision_geo_.cpp
)

ament_target_dependencies(vision_geo
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(vision_geo PRIVATE
  include
  /home/el-system/.local/opencv-4.10/include/opencv4
)

target_link_libraries(vision_geo
  /home/el-system/.local/opencv-4.10/lib/libopencv_calib3d.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_dnn.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_core.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_highgui.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_imgproc.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_videoio.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_imgcodecs.so
)

set_target_properties(vision_geo PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "/home/el-system/.local/opencv-4.10/lib"
)

install(TARGETS vision_geo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
