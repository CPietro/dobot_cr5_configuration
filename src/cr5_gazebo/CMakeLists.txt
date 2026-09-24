cmake_minimum_required(VERSION 3.8)
project(cr5_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_messages REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

install(DIRECTORY
  launch
  models
  urdf
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/zed_perception.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(oracle_frustum_culling SHARED src/oracle_frustum_culling.cpp)
ament_target_dependencies(oracle_frustum_culling
  rclcpp
  rclcpp_components
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  custom_messages
  visualization_msgs
)

rclcpp_components_register_node(oracle_frustum_culling PLUGIN "cr5_gazebo::OracleFrustumCulling" EXECUTABLE oracle_frustum_culling_node)

add_library(gazebo_bbox_publisher SHARED src/gazebo_bbox_publisher.cpp)
target_include_directories(gazebo_bbox_publisher PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(gazebo_bbox_publisher ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_bbox_publisher
  rclcpp
  gazebo_ros
  visualization_msgs
)

add_library(bbox_image_overlay SHARED src/bbox_image_overlay_node.cpp)
target_include_directories(bbox_image_overlay PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bbox_image_overlay ${OpenCV_LIBRARIES})
ament_target_dependencies(bbox_image_overlay
  rclcpp
  rclcpp_components
  sensor_msgs
  custom_messages
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rclcpp_components_register_node(bbox_image_overlay PLUGIN "cr5_gazebo::BboxImageOverlayNode" EXECUTABLE bbox_image_overlay_node)

install(TARGETS
  oracle_frustum_culling
  gazebo_bbox_publisher
  bbox_image_overlay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
