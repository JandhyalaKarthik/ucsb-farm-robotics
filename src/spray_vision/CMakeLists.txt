cmake_minimum_required(VERSION 3.8)
project(spray_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

option(SPRAY_VISION_USE_OPENCV "Use OpenCV VideoCapture for camera input" OFF)

add_executable(spray_detector_node
  src/camera_capture.cpp
  src/hailo_inference.cpp
  src/spray_detector_node.cpp
)
target_include_directories(spray_detector_node PRIVATE include)
ament_target_dependencies(spray_detector_node
  rclcpp
  std_msgs
)

if(SPRAY_VISION_USE_OPENCV)
  find_package(OpenCV REQUIRED)
  target_compile_definitions(spray_detector_node PRIVATE SPRAY_VISION_USE_OPENCV)
  target_link_libraries(spray_detector_node ${OpenCV_LIBS})
endif()

install(TARGETS
  spray_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
