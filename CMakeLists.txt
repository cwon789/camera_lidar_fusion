cmake_minimum_required(VERSION 3.8)
project(lidar_camera_colorizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_library(${PROJECT_NAME}_common
  src/calibration.cpp
)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_common
  geometry_msgs
  rclcpp
  std_msgs
  tf2
)
target_link_libraries(${PROJECT_NAME}_common
  ${OpenCV_LIBS}
)

add_executable(colorize_node src/colorize_node.cpp)
target_link_libraries(colorize_node ${PROJECT_NAME}_common ${OpenCV_LIBS})
ament_target_dependencies(colorize_node
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_executable(calib_tf_publisher src/calib_tf_publisher.cpp)
target_link_libraries(calib_tf_publisher ${PROJECT_NAME}_common ${OpenCV_LIBS})
ament_target_dependencies(calib_tf_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(demo_publisher src/demo_publisher.cpp)
target_link_libraries(demo_publisher ${PROJECT_NAME}_common ${OpenCV_LIBS})
ament_target_dependencies(demo_publisher
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    colorize_node
    calib_tf_publisher
    demo_publisher
    ${PROJECT_NAME}_common
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
