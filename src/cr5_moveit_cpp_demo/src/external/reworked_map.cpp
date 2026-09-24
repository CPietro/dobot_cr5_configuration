#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <custom_messages/msg/optimal_point.hpp>
#include <custom_messages/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/circumference.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <limits>

custom_messages::msg::Map map;

double cam_distance = 0.2;
int num_points = 64;
// int num_static_optimalities = 8; // Not used in the binary optimality model

double field_of_vision_z = 0.2;
double tolerance = 0.05; // how much to tolerate exceeding the field of view

void generateOptimalCircumferences(custom_messages::msg::Object& plant, double radius);
void checkWorkspace(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map);
void pointsInsideOtherObjects(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map);
std::vector<custom_messages::msg::Point> calculateCenters(custom_messages::msg::Object& plant);
void modifyMap(custom_messages::msg::Map& map);

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node")
    {
        // QoS configuration for the subscriber (latching)
        rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
        sub_qos.transient_local(); // Set durability to TRANSIENT_LOCAL

        // Create the subscriber for the 'Boing' topic
        map_subscriber_ = this->create_subscription<custom_messages::msg::Map>(
            "Boing",
            sub_qos,
            std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1)
        );

        // QoS configuration for the publisher (latching)
        rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
        pub_qos.transient_local();

        // Create the publisher for the 'reworked_map' topic
        map_publisher_ = this->create_publisher<custom_messages::msg::Map>("reworked_map", pub_qos);
    }

private:
    rclcpp::Subscription<custom_messages::msg::Map>::SharedPtr map_subscriber_;
    rclcpp::Publisher<custom_messages::msg::Map>::SharedPtr map_publisher_;

    void mapCallback(const custom_messages::msg::Map::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received map from Boing, starting reprocessing (silenced spam)...");
        map = *msg; // Make a copy of the received map
        modifyMap(map);
        map_publisher_->publish(map);
        // const int grid_size = 41;
        // char grid[grid_size][grid_size];

        // for (int i = 0; i < grid_size; ++i) {
        //     for (int j = 0; j < grid_size; ++j) {
        //         grid[i][j] = ' ';
        //     }
        // }

        // double scale = (double)(grid_size - 1) / (2.0 * cam_distance);
        // int center_grid = grid_size / 2;

        custom_messages::msg::Object optimal_points = map.objects[0];

        //std::cout <<map.objects.size() << std::endl;
        // std::cout <<map.objects[0].possible_trajectories.size() << std::endl;

        for (auto& circumference : optimal_points.possible_trajectories){
            for (const auto& point : circumference.circumference) {
                // int grid_x = static_cast<int>(round(center_grid + (point.x) * scale));
                // int grid_y = static_cast<int>(round(center_grid + (point.y) * scale));

                // if (grid_x >= 0 && grid_x < grid_size && grid_y >= 0 && grid_y < grid_size) {
                //     if (point.optimality == 1.0) {
                //         grid[grid_y][grid_x] = '1';
                //     } else {
                //         grid[grid_y][grid_x] = '0';
                //     }
                // }
                // std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << ", optimality: " << point.optimality << std::endl;
            }

            // std::cout << "2D Visualization (1 = Reachable, 0 = Not Reachable):\n";
            // for (int i = grid_size - 1; i >= 0; --i) {
            //     for (int j = 0; j < grid_size; ++j) {
            //         std::cout << grid[i][j] << " ";
            //     }
            //     std::cout << std::endl;
            // }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Map reprocessed and published on reworked_map.");
    }
};

void generateOptimalCircumferences(custom_messages::msg::Object& plant, double radius) {
    // num_static_optimalities is not relevant for binary optimality (0 or 1)
    std::vector<custom_messages::msg::Point> plant_centers = calculateCenters(plant);
    double angle_increment = 2.0 * M_PI / num_points;

    // std::cout << plant_centers.size() << std::endl;

    for (auto target_center : plant_centers){

        std::vector<custom_messages::msg::OptimalPoint> circumference_size(num_points);
        custom_messages::msg::Circumference trajectory;
        trajectory.circumference = circumference_size;

        for (int j = 0; j < num_points; ++j) {
            double angle = j * angle_increment;
            trajectory.circumference[j].x = target_center.x + radius * cos(angle);
            trajectory.circumference[j].y = target_center.y + radius * sin(angle);
            trajectory.circumference[j].z = target_center.z;
            trajectory.circumference[j].optimality = 1.0; // Initially set all points as optimal (1)
        }
        plant.possible_trajectories.push_back(trajectory);
    }
}

void checkWorkspace(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map) {  // check if points are reachable
    custom_messages::msg::BoundingBox ws_limits = map.work_space;
    custom_messages::msg::Point low = ws_limits.low_left;
    custom_messages::msg::Point top = ws_limits.top_right;

    for (auto& trajectory : plant.possible_trajectories){
        for (auto& point : trajectory.circumference) {
            // If a point is already marked as non-optimal, don't change it back to optimal
            if (point.optimality != 0.0) {
                if (point.x < low.x || point.x > top.x ||
                    point.y < low.y || point.y > top.y ||
                    point.z < low.z || point.z > top.z) {
                    point.optimality = 0.0; // Mark as non-optimal
                }
            }
        }
    }
}

void pointsInsideOtherObjects(custom_messages::msg::Object& plant, const custom_messages::msg::Map& map) { // check that points are not inside other objects (including itself)

    for (auto& object : map.objects){
        for (auto& trajectory : plant.possible_trajectories) {
            // Check if the current object being iterated through is the plant itself.
            // If it is, we usually don't want to mark points inside the plant as "non-optimal" due to the plant's own bounding box.
            // The original comment "if (&object != &plant)" suggests this intent, but it was commented out.
            // For a binary 0/1 optimality, it generally means "can the robot reach this point without colliding with ANYTHING".
            // If a point is inside the plant, and the plant is an obstacle *to itself* in this context, then it should be marked 0.
            // If the plant is NOT an obstacle to itself, then the check below should only apply to *other* objects.
            // For now, assuming it applies to all objects to be conservative (i.e., a point inside *any* object is not reachable).
            // To make it ignore the plant itself, uncomment the `if (&object != &plant)` line.
            // if (&object != &plant) {
                for (auto& point : trajectory.circumference) {
                    // If a point is already marked as non-optimal, don't change it back to optimal
                    if (point.optimality != 0.0) {
                        custom_messages::msg::Point low = object.shape.low_left;
                        custom_messages::msg::Point top = object.shape.top_right;
                        if (point.x >= low.x && point.x <= top.x &&
                            point.y >= low.y && point.y <= top.y &&
                            point.z >= low.z && point.z <= top.z) {
                            point.optimality = 0.0; // Mark as non-optimal
                        }
                    }
                }
            // }
        }
    }
}

std::vector<custom_messages::msg::Point> calculateCenters(custom_messages::msg::Object& plant) {
    custom_messages::msg::Point low_left = plant.shape.low_left;
    custom_messages::msg::Point top_right = plant.shape.top_right;
    // std::cout << "xx :" << low_left.x << " yy:" << low_left.y << " zz:" << top_right.x << std::endl;

    double height = top_right.z - low_left.z;
    std::vector<custom_messages::msg::Point> centers;

    if (height <= field_of_vision_z + tolerance) {
        // Only one center
        custom_messages::msg::Point center;
        center.x = top_right.x - (top_right.x - low_left.x) / 2.0;
        center.y = top_right.y - (top_right.y - low_left.y) / 2.0;
        center.z = top_right.z + (0.2) * field_of_vision_z;
        centers.push_back(center);
    } else {
        // Calculate the number of required divisions
        int num_divisions = height / field_of_vision_z;
        if (height - (num_divisions * field_of_vision_z) > tolerance) {
            num_divisions++;  // +1 because we truncate a part that will be the final < field_of_vision_y
        }

        for (int i = 0; i < num_divisions; i++) {
            custom_messages::msg::Point center;
            center.x = top_right.x - (top_right.x - low_left.x) / 2.0;
            center.y = top_right.y - (top_right.y - low_left.y) / 2.0;

            if (i != 0){
                center.z = top_right.z - (i + 0.5) * field_of_vision_z;
            } else {
                center.z = top_right.z + (0.25) * field_of_vision_z; // the last circle is raised compared to the plant
            }
            // std::cout << "x :" << center.x << " y:" << center.y << " z:" << center.z << std::endl;
            centers.push_back(center);
        }
    }
    return centers;
}

void modifyMap(custom_messages::msg::Map& map){
    for (auto& object : map.objects){
        //std::cout << object.target << std::endl;
        if (object.target){
            //std::cout << 1 << std::endl;

            // Calculate base diagonal and add it to radius
            custom_messages::msg::Point low_left = object.shape.low_left;
            custom_messages::msg::Point top_right = object.shape.top_right;
            double width = top_right.x - low_left.x;
            double depth = top_right.y - low_left.y;
            double base_diagonal = sqrt(width * width + depth * depth);
            
            double radius = cam_distance + base_diagonal / 2.0; // Add half diagonal as safety margin
            generateOptimalCircumferences(object, radius);
            checkWorkspace(object, map);
            pointsInsideOtherObjects(object, map);
        }
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // This must be called after your test code if you want the node to spin and receive messages.
    // If you only want to run the static test, you can keep it commented out.
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();

    return 0;
}