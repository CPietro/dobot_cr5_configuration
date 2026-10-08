#include "modules/plant_scanner.hpp"
#include <cmath>
#include <custom_messages/msg/circumference.hpp>

namespace cr5_demo {

// Constants
const double CAMERA_FOV_HORIZONTAL = 0.30; // Camera FOV radius in meters

// Logger for standalone functions that match original code
static rclcpp::Logger logger_ = rclcpp::get_logger("cr5_plant_scanner");

PlantScanner::PlantScanner(std::shared_ptr<rclcpp::Node> node) 
    : node_(node), movement_controller_(std::make_shared<MovementController>(node)) {
}

void PlantScanner::processCircumference(geometry_msgs::msg::Point& target_to_watch,
                                       const custom_messages::msg::Circumference& circumference,
                                       moveit::planning_interface::MoveGroupInterface& move_group,
                                       [[maybe_unused]] int trajectory_index) {
    RCLCPP_INFO(node_->get_logger(), "[SCANNING] Analyzing circumference with %zu points", circumference.circumference.size());
    
    // Convert points to ScanPoint structure
    std::vector<ScanPoint> scan_points = convertToScanPoints(circumference.circumference, target_to_watch);
    
    int optimal_count = 0;
    int unreachable_count = 0;
    int partial_count = 0;

    // Count optimal/unreachable points
    for (const auto& sp : scan_points) {
        if (sp.optimality == 1.0) { 
            optimal_count++;
        } else if (sp.optimality == 0.0) {
            unreachable_count++;
        } else {
            partial_count++;
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS] Circumference statistics:");
    RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS]   - Optimal points (reachable): %d", optimal_count);
    RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS]   - Unreachable points: %d", unreachable_count);
    if (partial_count > 0) {
        RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS]   - Partial points: %d", partial_count);
    }
    
    // Identify boundary points
    RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS] Identifying boundary points...");
    identifyBoundaryPoints(scan_points);
    
    // Count boundary points
    int boundary_count = 0;
    for (const auto& sp : scan_points) {
        if (sp.is_boundary && sp.optimality == 1.0) {
            boundary_count++;
        }
    }
    
    if (boundary_count > 0) {
        RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS] ✓ Found %d critical boundary points", boundary_count);
    } else {
        RCLCPP_INFO(node_->get_logger(), "[POINT-ANALYSIS] No boundary points identified");
    }
    
    // Phase 1: Try to visit boundary points
    if (boundary_count > 0) {
        RCLCPP_INFO(node_->get_logger(), " ");
        RCLCPP_INFO(node_->get_logger(), "[PHASE-1] ▶ Attempting to reach boundary points");
        RCLCPP_INFO(node_->get_logger(), "[PHASE-1] Objective: visit %d critical points", boundary_count);
        int visited_boundaries = 0;
        for (size_t i = 0; i < scan_points.size(); ++i) {
            if (scan_points[i].is_boundary && 
                scan_points[i].optimality == 1.0 && 
                !scan_points[i].covered) {
                RCLCPP_INFO(node_->get_logger(), "[PHASE-1] > Attempting boundary point #%zu: (%.3f, %.3f, %.3f)", 
                           i, scan_points[i].pose.position.x, scan_points[i].pose.position.y, scan_points[i].pose.position.z);
                
                if (movement_controller_->attemptToReachPoint(scan_points[i], move_group, i, node_)) {
                    visited_boundaries++;
                    // Mark nearby points as covered
                    int covered = markPointsAsCovered(scan_points[i], scan_points);
                    RCLCPP_INFO(node_->get_logger(), "[COVERAGE] >> ✓ Marked %d points as covered by camera FOV", covered);
                }
            }
        }
        RCLCPP_INFO(node_->get_logger(), "[PHASE-1] ✓ Completed - visited %d/%d boundary points", 
                   visited_boundaries, boundary_count);
    } else {
        RCLCPP_INFO(node_->get_logger(), "[PHASE-1] ⚠ Skipped - no boundary points found");
    }
    
    // Phase 2: Visit any remaining uncovered optimal points
    RCLCPP_INFO(node_->get_logger(), " ");
    RCLCPP_INFO(node_->get_logger(), "[PHASE-2] ▶ Scanning remaining uncovered points");
    scanUncoveredPoints(scan_points, target_to_watch, move_group);
}

void PlantScanner::identifyBoundaryPoints(std::vector<ScanPoint>& scan_points) {
    size_t n = scan_points.size();
    for (size_t i = 0; i < n; ++i) {
        if (scan_points[i].optimality == 1.0) { // Optimal point
            // Check previous and next points (circular)
            size_t prev = (i - 1 + n) % n;
            size_t next = (i + 1) % n;
            
            if (scan_points[prev].optimality == 0.0 || 
                scan_points[next].optimality == 0.0) {
                scan_points[i].is_boundary = true;
            }
        }
    }
}

void PlantScanner::scanUncoveredPoints(std::vector<ScanPoint>& scan_points,
                                      [[maybe_unused]] geometry_msgs::msg::Point& target_to_watch,
                                      moveit::planning_interface::MoveGroupInterface& move_group) {
    // Find all uncovered optimal points
    std::vector<size_t> uncovered_indices;
    for (size_t i = 0; i < scan_points.size(); ++i) {
        if (scan_points[i].optimality == 1.0 && !scan_points[i].covered) {
            uncovered_indices.push_back(i);
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "[PHASE-2] Found %zu uncovered points to scan", uncovered_indices.size());
    
    if (uncovered_indices.empty()) {
        RCLCPP_INFO(node_->get_logger(), "[PHASE-2] ✓ All optimal points are already covered!");
        return;
    }
    
    // Process uncovered points
    RCLCPP_INFO(node_->get_logger(), "[PHASE-2] Starting scan of remaining points...");
    int successful_scans = 0;
    int failed_scans = 0;
    
    for (size_t idx : uncovered_indices) {
        if (!scan_points[idx].covered) { // Double check as it might have been covered by previous scan
            RCLCPP_INFO(node_->get_logger(), "[PHASE-2] > Attempting uncovered point #%zu: (%.3f, %.3f, %.3f)", 
                       idx, scan_points[idx].pose.position.x, scan_points[idx].pose.position.y, scan_points[idx].pose.position.z);

            if (movement_controller_->attemptToReachPoint(scan_points[idx], move_group, idx, node_)) {
                successful_scans++;
                RCLCPP_INFO(node_->get_logger(), "[PHASE-2] > ✓ Success for point #%zu", idx);
                
                // Mark nearby points as covered
                int covered = markPointsAsCovered(scan_points[idx], scan_points);
                RCLCPP_INFO(node_->get_logger(), "[COVERAGE] >> ✓ Marked %d points as covered by camera FOV", covered);
            } else {
                failed_scans++;
                RCLCPP_WARN(node_->get_logger(), "[PHASE-2] > ✗ Failure for point #%zu", idx);
            }
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "[PHASE-2] ✓ Completed - %d successes, %d failures", 
               successful_scans, failed_scans);
    
    // Log final coverage statistics
    int total_optimal = 0;
    int covered = 0;
    for (const auto& sp : scan_points) {
        if (sp.optimality == 1.0) {
            total_optimal++;
            if (sp.covered) {
                covered++;
            }
        }
    }
    
    double coverage_percentage = total_optimal > 0 ? 
                                (static_cast<double>(covered) / total_optimal) * 100.0 : 0.0;
    
    RCLCPP_INFO(node_->get_logger(), " ");
    RCLCPP_INFO(node_->get_logger(), "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓");
    RCLCPP_INFO(node_->get_logger(), "             FINAL COVERAGE REPORT");
    RCLCPP_INFO(node_->get_logger(), "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓");
    RCLCPP_INFO(node_->get_logger(), "[RESULT] Points covered: %d/%d (%.1f%%)", 
               covered, total_optimal, coverage_percentage);
    
    if (covered < total_optimal) {
        RCLCPP_WARN(node_->get_logger(), "[WARNING] %d points remain UNCOVERED:", total_optimal - covered);
        for (size_t i = 0; i < scan_points.size(); ++i) {
            if (scan_points[i].optimality == 1.0 && !scan_points[i].covered) {
                RCLCPP_WARN(node_->get_logger(), "[UNCOVERED]   - Point #%zu: (%.3f, %.3f, %.3f)",
                           i, scan_points[i].pose.position.x, scan_points[i].pose.position.y, scan_points[i].pose.position.z);
            }
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓");
}

geometry_msgs::msg::Pose PlantScanner::createPosePointingToCenter(
    const custom_messages::msg::OptimalPoint& point,
    const geometry_msgs::msg::Point& center) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.point.x;
    pose.position.y = point.point.y;
    pose.position.z = point.point.z;
    
    // Calculate orientation to point towards center
    tf2::Vector3 from(point.point.x, point.point.y, point.point.z);
    tf2::Vector3 to(center.x, center.y, center.z);
    tf2::Vector3 direction = to - from;
    direction.normalize();
    
    // Build orthogonal frame
    tf2::Vector3 z_axis = direction;
    tf2::Vector3 arbitrary(0, 0, 1);
    if (std::abs(z_axis.getZ()) > 0.9) {
        arbitrary = tf2::Vector3(0, 1, 0);
    }
    
    tf2::Vector3 x_axis = z_axis.cross(arbitrary);
    x_axis.normalize();
    tf2::Vector3 y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    
    tf2::Matrix3x3 rotation(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z()
    );
    
    tf2::Quaternion q;
    rotation.getRotation(q);

    // Add 180° rotation around end effector Z-axis
    tf2::Quaternion z_rotation;
    z_rotation.setRPY(0, 0, M_PI); // 180 degrees in radians around Z
    
    // Combine rotations: first pointing to center, then 180° around Z
    q = q * z_rotation;
    
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}


//useless for now but who knows. it might be useful in the future
double PlantScanner::calculateDistance(const custom_messages::msg::OptimalPoint& p1, 
                                     const custom_messages::msg::OptimalPoint& p2) {
    double dx = p1.point.x - p2.point.x;
    double dy = p1.point.y - p2.point.y;
    double dz = p1.point.z - p2.point.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

int PlantScanner::markPointsAsCovered(ScanPoint& current_point, std::vector<ScanPoint>& all_points) {
    int covered_count = 1; // Current point itself
    current_point.covered = true;
    
    // Mark nearby points as covered based on camera FOV
    for (auto& point : all_points) {
        if (!point.covered && point.optimality == 1.0) {
            double dx = current_point.pose.position.x - point.pose.position.x;
            double dy = current_point.pose.position.y - point.pose.position.y;
            double dz = current_point.pose.position.z - point.pose.position.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Simple FOV model: if point is within FOV range, mark as covered
            if (dist < CAMERA_FOV_HORIZONTAL) {
                point.covered = true;
                covered_count++;
                RCLCPP_DEBUG(node_->get_logger(), "        Covered point at distance %.3f", dist);
            }
        }
    }
    
    return covered_count;
}

std::vector<ScanPoint> PlantScanner::convertToScanPoints(
    const std::vector<custom_messages::msg::OptimalPoint>& optimal_points,
    const geometry_msgs::msg::Point& target_center) {
    std::vector<ScanPoint> scan_points;
    
    for (size_t i = 0; i < optimal_points.size(); ++i) {
        ScanPoint sp;
        sp.pose = createPosePointingToCenter(optimal_points[i], target_center);
        sp.is_boundary = false;
        sp.covered = false;
        sp.optimality = optimal_points[i].optimality;
        sp.index = i;
        scan_points.push_back(sp);
    }
    
    return scan_points;
}

} // namespace cr5_demo