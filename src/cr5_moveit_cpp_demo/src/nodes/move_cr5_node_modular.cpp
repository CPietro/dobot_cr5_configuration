#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <custom_messages/msg/map.hpp>
#include <cmath>

// Include our modular components
#include "modules/plant_processor.hpp"
#include "modules/scene_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("move_cr5_node_modular");
    
    RCLCPP_INFO(node->get_logger(), "=========================================================");
    RCLCPP_INFO(node->get_logger(), "                CR5 MOTION CONTROLLER                   ");
    RCLCPP_INFO(node->get_logger(), "                   SYSTEM STARTUP                       ");
    RCLCPP_INFO(node->get_logger(), "=========================================================");
    
    // Initialize MoveIt interfaces
    RCLCPP_INFO(node->get_logger(), "[PHASE 1] Initializing MoveIt interfaces...");
    moveit::planning_interface::MoveGroupInterface move_group(node, "cr5_group");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    RCLCPP_INFO(node->get_logger(), "[PHASE 1] ✓ MoveIt interfaces initialized successfully");

    // Configure planning parameters
    RCLCPP_INFO(node->get_logger(), "[PHASE 2] Configuring planning parameters...");
    move_group.setPlanningTime(0.5); // 0.5 seconds max planning time
    move_group.setNumPlanningAttempts(5); // Try up to 5 times

    // Set tolerances - 2cm position tolerance as requested
    move_group.setGoalPositionTolerance(0.02); // 2cm
    move_group.setGoalOrientationTolerance(0.1); // ~5.7 degrees
    move_group.setGoalJointTolerance(0.01); // Joint tolerance
    
    RCLCPP_INFO(node->get_logger(), "[PHASE 2] MoveIt configuration completed:");
    RCLCPP_INFO(node->get_logger(), "         - Planning time: %.1f seconds", move_group.getPlanningTime());
    RCLCPP_INFO(node->get_logger(), "         - Position tolerance: %.3f m (%.0f cm)", move_group.getGoalPositionTolerance(), move_group.getGoalPositionTolerance() * 100);
    RCLCPP_INFO(node->get_logger(), "         - Orientation tolerance: %.3f rad (%.1f°)", move_group.getGoalOrientationTolerance(), move_group.getGoalOrientationTolerance() * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "         - Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "         - End effector: %s", move_group.getEndEffectorLink().c_str());
    
    // Initialize modular components
    RCLCPP_INFO(node->get_logger(), "[PHASE 3] Initializing system components...");
    auto plant_processor = std::make_shared<cr5_demo::PlantProcessor>(node);
    auto scene_manager = std::make_shared<cr5_demo::SceneManager>(node);
    RCLCPP_INFO(node->get_logger(), "[PHASE 3] ✓ System components initialized");

    // QoS configuration for subscriber (latching)
    RCLCPP_INFO(node->get_logger(), "[PHASE 4] Configuring map subscriber...");
    rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
    sub_qos.transient_local();
    
    // Subscribe to reworked_map topic
    auto map_subscriber = node->create_subscription<custom_messages::msg::Map>(
        "reworked_map",
        sub_qos,
        [&](const custom_messages::msg::Map::SharedPtr msg) {
            plant_processor->mapCallback(msg, move_group, planning_scene_interface);
        }
    );
    
    RCLCPP_INFO(node->get_logger(), "---------------------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "    ✓ INITIALIZATION COMPLETED SUCCESSFULLY            ");
    RCLCPP_INFO(node->get_logger(), "    >> System ready - Waiting for maps to process...");
    RCLCPP_INFO(node->get_logger(), "---------------------------------------------------------");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}