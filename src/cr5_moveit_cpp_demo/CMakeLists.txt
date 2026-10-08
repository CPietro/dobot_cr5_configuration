cmake_minimum_required(VERSION 3.8)
project(cr5_moveit_cpp_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tl_expected REQUIRED)
if(NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

find_package(PythonLibs REQUIRED) # Necessario per i file di launch Python
find_package(ament_cmake_python REQUIRED) # Necessario per il supporto di Python Ament

find_package(ament_cmake REQUIRED)
find_package(custom_messages REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

# Add include directory for our headers
include_directories(include)

# Aggiungi l'eseguibile
add_executable(move_cr5_node src/nodes/move_cr5_node.cpp)
add_executable(move_cr5_node_modular 
    src/nodes/move_cr5_node_modular.cpp
    src/modules/movement_controller.cpp
    src/modules/plant_scanner.cpp
    src/modules/scene_manager.cpp
    src/modules/plant_processor.cpp
)
add_executable(reworked_map_node src/external/reworked_map.cpp)
add_executable(fake_zed_node src/external/fake_zed.cpp)
add_executable(reconnaissance_node src/external/reconnaissance.cpp)
add_executable(matlab_slave_node src/external/matlab_slave_node.cpp)
#add_executable(follow_target_node src/external/follow_target.cpp)

# Link delle dipendenze al target
ament_target_dependencies(move_cr5_node_modular
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)

ament_target_dependencies(move_cr5_node
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)

ament_target_dependencies(reworked_map_node
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)

ament_target_dependencies(fake_zed_node
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)

ament_target_dependencies(reconnaissance_node
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)

ament_target_dependencies(matlab_slave_node
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)

ament_target_dependencies(move_cr5_node
rclcpp
moveit_core
moveit_ros_planning_interface
moveit_ros_planning
moveit_ros_robot_interaction
moveit_ros_move_group
geometry_msgs
shape_msgs
moveit_msgs
tf2
tf2_geometry_msgs
custom_messages
)


#ament_target_dependencies(follow_target_node
#rclcpp
#moveit_core
#moveit_ros_planning_interface
#moveit_ros_planning
#moveit_ros_robot_interaction
#moveit_ros_move_group
#geometry_msgs
#shape_msgs
#moveit_msgs
#tf2
#tf2_geometry_msgs
#custom_messages
#)

# Installa l'eseguibile
install(TARGETS
move_cr5_node
move_cr5_node_modular
reworked_map_node
fake_zed_node
reconnaissance_node
matlab_slave_node
#follow_target_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()









