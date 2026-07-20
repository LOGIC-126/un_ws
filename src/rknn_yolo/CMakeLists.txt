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
find_package(vision_msgs REQUIRED)   # 新增：用于 Detection2DArray

# RKNN 头文件路径
include_directories(
  /home/orangepi/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/lib)

# ========== 原 YOLO 检测节点 ==========
add_executable(yolo_detector_node
  src/yolo_detector_node.cpp
)

ament_target_dependencies(yolo_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  vision_msgs        # 新增
)

target_link_libraries(yolo_detector_node
  rknnrt
  ${OpenCV_LIBS}
)

# ========== 摄像头发布节点（保持原样）==========
add_executable(camera_publisher_node
  src/camera_node.cpp
)

ament_target_dependencies(camera_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_publisher_node
  ${OpenCV_LIBS}
)

# 安装规则
install(TARGETS yolo_detector_node camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()