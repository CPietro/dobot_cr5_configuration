#include "modules/plant_processor.hpp"

namespace cr5_demo {

// Static variable to track first scan
static bool first_scan = true;

PlantProcessor::PlantProcessor(std::shared_ptr<rclcpp::Node> node) 
    : node_(node),
      scene_manager_(std::make_shared<SceneManager>(node)),
      plant_scanner_(std::make_shared<PlantScanner>(node)) {
}

void PlantProcessor::mapCallback(const custom_messages::msg::Map::SharedPtr msg,
                                moveit::planning_interface::MoveGroupInterface& move_group,
                                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    RCLCPP_INFO(node_->get_logger(), " ");
    RCLCPP_INFO(node_->get_logger(), "=========================================================");
    RCLCPP_INFO(node_->get_logger(), "                NEW MAP RECEIVED                        ");
    RCLCPP_INFO(node_->get_logger(), "=========================================================");
    RCLCPP_INFO(node_->get_logger(), "[MAP] Received map from 'reworked_map'");
    RCLCPP_INFO(node_->get_logger(), "[MAP] Total objects in map: %zu", msg->objects.size());
    
    current_map_ = *msg;
    
    // Count target plants
    int target_count = 0;
    int obstacle_count = 0;
    for (const auto& obj : current_map_.objects) {
        if (obj.target) 
            target_count++;
        else 
            obstacle_count++;
    }
    RCLCPP_INFO(node_->get_logger(), "[ANALYSIS] Target plants to scan: %d", target_count);
    RCLCPP_INFO(node_->get_logger(), "[ANALYSIS] Obstacles detected: %d", obstacle_count);
    
    if (target_count == 0) {
        RCLCPP_WARN(node_->get_logger(), "[WARNING] No target plants found in the map!");
        return;
    }
    
    // Add obstacles to planning scene
    RCLCPP_INFO(node_->get_logger(), " ");
    RCLCPP_INFO(node_->get_logger(), "---------------------------------------------------------");
    RCLCPP_INFO(node_->get_logger(), "[SETUP] Starting planning environment configuration");
    scene_manager_->addObstaclesToScene(move_group, planning_scene_interface, current_map_);
    
    // Process each target plant
    RCLCPP_INFO(node_->get_logger(), "[EXECUTION] Starting target plant processing");
    RCLCPP_INFO(node_->get_logger(), "---------------------------------------------------------");
    processTargetPlants(move_group);
    
    RCLCPP_INFO(node_->get_logger(), " ");
    RCLCPP_INFO(node_->get_logger(), "=========================================================");
    RCLCPP_INFO(node_->get_logger(), "        ✓ MAP PROCESSING COMPLETED                      ");
    RCLCPP_INFO(node_->get_logger(), "=========================================================");
}

void PlantProcessor::processTargetPlants(moveit::planning_interface::MoveGroupInterface& move_group) {
    int plant_number = 0;
    for (const auto& object : current_map_.objects) {
        if (object.target) {
            plant_number++;
            RCLCPP_INFO(node_->get_logger(), " ");
            RCLCPP_INFO(node_->get_logger(), "█████████████████████████████████████████████████████████");
            RCLCPP_INFO(node_->get_logger(), "          PROCESSING TARGET PLANT #%d", plant_number);
            RCLCPP_INFO(node_->get_logger(), "█████████████████████████████████████████████████████████");
            
            RCLCPP_INFO(node_->get_logger(), "[PLANT-%d] Bounding box information:", plant_number);
            RCLCPP_INFO(node_->get_logger(), "[PLANT-%d]   Bottom-left corner: (%.3f, %.3f, %.3f)", plant_number,
                       object.shape.low_left.x, object.shape.low_left.y, object.shape.low_left.z);
            RCLCPP_INFO(node_->get_logger(), "[PLANT-%d]   Top-right corner: (%.3f, %.3f, %.3f)", plant_number,
                       object.shape.top_right.x, object.shape.top_right.y, object.shape.top_right.z);
            
            double volume_x = object.shape.top_right.x - object.shape.low_left.x;
            double volume_y = object.shape.top_right.y - object.shape.low_left.y;
            double volume_z = object.shape.top_right.z - object.shape.low_left.z;
            RCLCPP_INFO(node_->get_logger(), "[PLANT-%d]   Dimensions: %.3f x %.3f x %.3f m", plant_number, volume_x, volume_y, volume_z);
            
            RCLCPP_INFO(node_->get_logger(), "[PLANT-%d] Available trajectories: %zu", plant_number,
                       object.possible_trajectories.size());
            
            if (object.possible_trajectories.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[PLANT-%d] ⚠️  WARNING: No trajectories available!", plant_number);
                continue;
            }
            
            // Process each circumference for this plant
            for (size_t traj_idx = 0; traj_idx < object.possible_trajectories.size(); ++traj_idx) {
                RCLCPP_INFO(node_->get_logger(), " ");
                RCLCPP_INFO(node_->get_logger(), "[PLANT-%d] ═══ TRAJECTORY %zu/%zu ═══", plant_number,
                           traj_idx + 1, object.possible_trajectories.size());

                // Calculate plant center for scanning
                geometry_msgs::msg::Point target_to_watch = calculateScanCenter(object, first_scan);
                
                if (first_scan) {
                    first_scan = false;
                    RCLCPP_INFO(node_->get_logger(), "[PLANT-%d] Calculated center (first scan): (%.3f, %.3f, %.3f)", plant_number,
                               target_to_watch.x, target_to_watch.y, target_to_watch.z);
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[PLANT-%d] Used center (subsequent scans): (%.3f, %.3f, %.3f)", plant_number,
                               target_to_watch.x, target_to_watch.y, target_to_watch.z);
                }
                
                // Process this circumference/trajectory
                plant_scanner_->processCircumference(target_to_watch, 
                                                    object.possible_trajectories[traj_idx], 
                                                    move_group, 
                                                    traj_idx + 1);
            }
            first_scan = true; // Reset for next plant
            RCLCPP_INFO(node_->get_logger(), " ");
            RCLCPP_INFO(node_->get_logger(), "█████████████████████████████████████████████████████████");
            RCLCPP_INFO(node_->get_logger(), "        ✓ PLANT #%d COMPLETED SUCCESSFULLY", plant_number);
            RCLCPP_INFO(node_->get_logger(), "█████████████████████████████████████████████████████████");
        }
    }
}

geometry_msgs::msg::Point PlantProcessor::calculateScanCenter(const custom_messages::msg::Object& object,
                                                             bool use_first_scan) {
    geometry_msgs::msg::Point target_to_watch;
    target_to_watch.x = (object.shape.low_left.x + object.shape.top_right.x) / 2.0;
    target_to_watch.y = (object.shape.low_left.y + object.shape.top_right.y) / 2.0;

    if (use_first_scan) {
        target_to_watch.z = (object.shape.low_left.z + object.shape.top_right.z) / 2.0;
    } else {
        // Use z of first point for setting height
        if (!object.possible_trajectories.empty() && !object.possible_trajectories[0].circumference.empty()) {
            target_to_watch.z = object.possible_trajectories[0].circumference[0].point.z;
        } else {
            target_to_watch.z = (object.shape.low_left.z + object.shape.top_right.z) / 2.0;
        }
    }

    return target_to_watch;
}

} // namespace cr5_demo