cmake_minimum_required(VERSION 3.8)
project(rknn_yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# RKNN 头文件路径
include_directories(
  /home/orangepi/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/lib)

# 编译可执行文件
add_executable(yolo_detector_node
  src/yolo_detector_node.cpp
)

# 链接 ROS 2 依赖
ament_target_dependencies(yolo_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# 链接 RKNN 和 OpenCV
target_link_libraries(yolo_detector_node
  rknnrt
  ${OpenCV_LIBS}
)

# 安装规则
install(TARGETS yolo_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()