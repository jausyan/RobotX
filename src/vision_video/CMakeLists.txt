cmake_minimum_required(VERSION 3.10)
project(vision_video)

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
option(ENABLE_OPENVINO "Enable OpenVINO runtime linkage" OFF)
if(ENABLE_OPENVINO)
  find_package(
    OpenVINO REQUIRED COMPONENTS Runtime
    PATHS /home/el-system/.local/lib/python3.10/site-packages/openvino/cmake
  )
endif()
add_executable(vision_video
  src/vision_video.cpp
  utils/vision_video_.cpp
)
ament_target_dependencies(vision_video rclcpp std_msgs geometry_msgs sensor_msgs)

target_include_directories(vision_video PRIVATE
  include
  /home/el-system/.local/opencv-4.10/include/opencv4
)
target_link_libraries(vision_video
  /home/el-system/.local/opencv-4.10/lib/libopencv_dnn.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_calib3d.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_features2d.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_flann.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_core.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_highgui.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_imgproc.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_videoio.so
  /home/el-system/.local/opencv-4.10/lib/libopencv_imgcodecs.so
)
# Burn 4.10 lib dir into the binary so the dynamic linker uses 4.10 at runtime
set_target_properties(vision_video PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "/home/el-system/.local/opencv-4.10/lib"
)
if(ENABLE_OPENVINO)
  target_link_libraries(vision_video openvino::runtime::c)
  target_compile_definitions(vision_video PRIVATE ENABLE_OPENVINO_RUNTIME=1)
  target_compile_options(
    vision_video PRIVATE
    -U_GLIBCXX_USE_CXX11_ABI
    -D_GLIBCXX_USE_CXX11_ABI=1
  )
endif()

install(TARGETS
  vision_video
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
