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

# ── OpenCV: the custom 4.10 build if present (laptop), else the system OpenCV (e.g. 4.6 on the drone) ──
unset(OpenCV_DIR CACHE)
set(CUSTOM_OPENCV_DIR "$ENV{HOME}/.local/opencv-4.10")
set(OPENCV_COMPONENTS core dnn imgproc imgcodecs highgui videoio calib3d)
if(EXISTS "${CUSTOM_OPENCV_DIR}/lib/cmake/opencv4/OpenCVConfig.cmake")
  find_package(OpenCV REQUIRED COMPONENTS ${OPENCV_COMPONENTS}
    PATHS "${CUSTOM_OPENCV_DIR}/lib/cmake/opencv4" NO_DEFAULT_PATH)
  set(VISION_GEO_RPATH "${CUSTOM_OPENCV_DIR}/lib")
else()
  find_package(OpenCV REQUIRED COMPONENTS ${OPENCV_COMPONENTS})
  set(VISION_GEO_RPATH "")
endif()
message(STATUS "vision_geo: OpenCV ${OpenCV_VERSION} (${OpenCV_DIR})")

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

target_include_directories(vision_geo PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_geo ${OpenCV_LIBS})

# ── OpenVINO (C API): pip package (laptop) or apt libopenvino-dev (drone) ─────────────────
# ON by default; if OpenVINO is not found it builds without it (use_openvino must stay false).
option(ENABLE_OPENVINO "Enable OpenVINO runtime linkage" ON)
if(ENABLE_OPENVINO)
  find_package(OpenVINO QUIET COMPONENTS Runtime PATHS
    "$ENV{HOME}/.local/lib/python3.10/site-packages/openvino/cmake"
    "$ENV{HOME}/.local/lib/python3.12/site-packages/openvino/cmake")
  if(OpenVINO_FOUND)
    target_link_libraries(vision_geo openvino::runtime::c)
    target_compile_definitions(vision_geo PRIVATE ENABLE_OPENVINO_RUNTIME=1)
    # the pip OpenVINO target forces the old C++ ABI (_GLIBCXX_USE_CXX11_ABI=0), which breaks
    # linking with ROS / OpenCV; we only use the C API, so keep the new ABI
    target_compile_options(vision_geo PRIVATE -U_GLIBCXX_USE_CXX11_ABI -D_GLIBCXX_USE_CXX11_ABI=1)
    # runtime must find libopenvino_c.so wherever it is installed (pip dir is not on the linker path)
    get_target_property(OV_C_LIB openvino::runtime::c IMPORTED_LOCATION_RELEASE)
    if(OV_C_LIB)
      get_filename_component(OV_LIB_DIR "${OV_C_LIB}" DIRECTORY)
      list(APPEND VISION_GEO_RPATH "${OV_LIB_DIR}")
    endif()
    message(STATUS "vision_geo: OpenVINO ${OpenVINO_VERSION} enabled")
  else()
    message(WARNING "vision_geo: OpenVINO not found - building without it")
  endif()
endif()

set_target_properties(vision_geo PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${VISION_GEO_RPATH}"
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
