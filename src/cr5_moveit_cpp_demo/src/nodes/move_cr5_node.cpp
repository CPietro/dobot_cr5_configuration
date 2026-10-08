/*
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>  
#include <moveit_msgs/msg/collision_object.hpp>                         
#include <shape_msgs/msg/solid_primitive.hpp>                           
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <vector>
#include <cmath>

//#include <moveit/robot_state/robot_state.h>    serve per settare lo statato del braccio a quello della simulazione rviz

using namespace std::chrono_literals;

struct PlantWithCenter
{
  moveit_msgs::msg::CollisionObject plant_shape;
  geometry_msgs::msg::Point center;
};


std::vector<PlantWithCenter> plants; //array delle piante

PlantWithCenter target;  // pianta da scannerizzare

double radius = 0.1; // distanza minima dalla pianta (terrà conto anche della diagonale del box pianta)

double diagonal; // la diagonale del box pianta

std::array<float, 2> high_pos_x = {-0.10,0.10};  //{coordinata x piu negativa,coordinata x piu positiva}

std::array<float, 2> high_pos_y = {-0.15,0.15};  //{coordinata x piu negativa,coordinata x piu positiva}

std::array<float, 2> high_pos_z = {1,0.80};  //{coordinata x piu vicina a terra,coordinata x piu lontana da terra}





std::array<float, 3> calculate_dim_plant(std::array<float, 2> high_pos_x,std::array<float, 2> high_pos_y,std::array<float, 2> high_pos_z);
geometry_msgs::msg::Point calculate_obstacle_center(std::array<float, 2> high_pos_x,std::array<float, 2> high_pos_y,std::array<float, 2> high_pos_z);
PlantWithCenter add_obstacle(std::string name, std::array<float, 3> dim, moveit::planning_interface::PlanningSceneInterface planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group);
std::vector<geometry_msgs::msg::Pose> create_semi_circ(PlantWithCenter target, double radius, double angle_start, double angle_end, moveit::planning_interface::MoveGroupInterface& move_group);
int execute_plant_scan(std::vector<geometry_msgs::msg::Pose> waypoints, moveit::planning_interface::MoveGroupInterface& move_group);







int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("cr5_moveit_cpp_planner");
    moveit::planning_interface::MoveGroupInterface move_group(node, "cr5_group");

    // Aggiungiamo anche l'interfaccia alla scena (serve per l'ostacolo, basta un'istanza per tutti gli ostacoli) 
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());



    /////-------------------- SOLO PER TESTING PEERMETTE DI AGGIUNGERE UN PIANO DI SICUREZZA----------------------------
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = move_group.getPlanningFrame();  // serve a prendere la posizione della base del robot per capire dove posizionare l'oggetto
    obstacle.id = "floor";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX; // tipo dell'oggetto

    radius += diagonal; // aggiorno il raggio con la diagonale del box pianta

    primitive.dimensions = {2, 2, 0.5};  // dimensioni oggetto

    geometry_msgs::msg::Point obstacle_center = calculate_obstacle_center({-1,1}, {-1,1}, {0, -0.5});  //AL MOMENTO TUTTI GLI OSTACOLI HANNO LO STESSO CENTRO
    geometry_msgs::msg::Pose box_pose; // centro del solido
    box_pose.position.x = obstacle_center.x;
    box_pose.position.y = obstacle_center.y;
    box_pose.position.z = obstacle_center.z;
    box_pose.orientation.w = 1.0;

    obstacle.primitives.push_back(primitive);  // aggiunge all'oggetto la sua forma appena descritta
    obstacle.primitive_poses.push_back(box_pose); // aggiunge all'oggetto la sua posizione appena descritta
    obstacle.operation = obstacle.ADD;

    planning_scene_interface.applyCollisionObjects({obstacle});
    rclcpp::sleep_for(1s);  // diamo tempo per sincronizzarsi
    //RCLCPP_INFO(node->get_logger(), "Ostacolo aggiunto alla scena.");
    // ------------------------------

    // Aspettiamo un attimo che la scena sia pronta
    rclcpp::sleep_for(1s);
    /////-------------------- SOLO PER TESTING PEERMETTE DI AGGIUNGERE UN PIANO DI SICUREZZA----------------------------





    std::array<float, 3> dim = calculate_dim_plant(high_pos_x, high_pos_y, high_pos_z);
    
    PlantWithCenter plant = add_obstacle("plant1", dim, planning_scene_interface, move_group);



    rclcpp::shutdown();
    return 0;
}










std::array<float, 3> calculate_dim_plant(std::array<float, 2> high_pos_x,std::array<float, 2> high_pos_y,std::array<float, 2> high_pos_z){
    
    float lenght = abs(high_pos_x[0] - high_pos_x[1]); // x
    float depth = abs(high_pos_y[0] - high_pos_y[1]); // y
    float height = abs(high_pos_z[0] - high_pos_z[1]); // z
    diagonal = std::sqrt(std::pow(lenght, 2) + std::pow(depth, 2) + std::pow(height, 2))/2;

    std::cout << diagonal << std::endl;

    std::array<float, 3> dim = {lenght, depth, height};

    return dim;
}

geometry_msgs::msg::Point calculate_obstacle_center(std::array<float, 2> high_pos_x,std::array<float, 2> high_pos_y,std::array<float, 2> high_pos_z){
    
    geometry_msgs::msg::Point center;
    
    center.x = high_pos_x[0] + (high_pos_x[1] - high_pos_x[0])/2;
    center.y = high_pos_y[0] + (high_pos_y[1] - high_pos_y[0])/2;
    center.z = high_pos_z[0] + (high_pos_z[1] - high_pos_z[0])/2;

    return center;
}


PlantWithCenter add_obstacle(std::string name, std::array<float, 3> dim, moveit::planning_interface::PlanningSceneInterface planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group){

    // ----- AGGIUNTA OSTACOLO -----
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = move_group.getPlanningFrame();  // serve a prendere la posizione della base del robot per capire dove posizionare l'oggetto
    obstacle.id = name;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX; // tipo dell'oggetto

    radius += diagonal; // aggiorno il raggio con la diagonale del box pianta

    primitive.dimensions = {dim[0], dim[1], dim[2]};  // dimensioni oggetto

    geometry_msgs::msg::Point obstacle_center = calculate_obstacle_center(high_pos_x, high_pos_y, high_pos_z);  //AL MOMENTO TUTTI GLI OSTACOLI HANNO LO STESSO CENTRO
    geometry_msgs::msg::Pose box_pose; // centro del solido
    box_pose.position.x = obstacle_center.x;
    box_pose.position.y = obstacle_center.y;
    box_pose.position.z = obstacle_center.z;
    box_pose.orientation.w = 1.0;

    obstacle.primitives.push_back(primitive);  // aggiunge all'oggetto la sua forma appena descritta
    obstacle.primitive_poses.push_back(box_pose); // aggiunge all'oggetto la sua posizione appena descritta
    obstacle.operation = obstacle.ADD;

    planning_scene_interface.applyCollisionObjects({obstacle});

    // creazione dati dell'ostacolo
    PlantWithCenter plant;
    plant.plant_shape = obstacle;
    plant.center = obstacle_center;

    plants.push_back(plant);  // aggiunge all'array il nostro ostacolo
    
    rclcpp::sleep_for(1s);  // diamo tempo per sincronizzarsi
    //RCLCPP_INFO(node->get_logger(), "Ostacolo aggiunto alla scena.");
    // ------------------------------

    // Aspettiamo un attimo che la scena sia pronta
    rclcpp::sleep_for(1s);

    return plant;
}



std::vector<geometry_msgs::msg::Pose> create_semi_circ(PlantWithCenter target, double radius, double angle_start, double angle_end, moveit::planning_interface::MoveGroupInterface& move_group){
    std::vector<geometry_msgs::msg::Pose> waypoints;
    tf2::Quaternion q;

    int num_points = 10;  // più punti = più fluido
    double angle_step = (angle_end - angle_start) / num_points;
    bool first_is_available = false;

    while (!first_is_available){
        for (int i = 0; i <= num_points; ++i)
        {
            double theta = angle_start + i * angle_step;  // angolo punto corrente
            geometry_msgs::msg::Pose pose;

            double cx = target.center.x;
            double cy = target.center.y;
            double cz = target.center.z;

            pose.position.x = cx + radius * std::cos(theta);
            pose.position.y = cy + radius * std::sin(theta);
            pose.position.z = cz;  // mantieni altezza costante


        // ---------- CALCOLO ORIENTAMENTO END EFFECTOR -------------------
            tf2::Vector3 from(pose.position.x, pose.position.y, pose.position.z);  // vettore che punta l'end effector
            tf2::Vector3 to(cx, cy, cz); // vettore che punta il centro della pianta
            tf2::Vector3 direction = to - from;
            direction.normalize();  // vettore unitario

            // asse z definito da noi (vettore che punta all'oggetto)
            tf2::Vector3 z_axis = direction;

            // l’asse Z (arbitrario ma perpendicolare), uso l'asse z per evitare che cambi durante la scansione orizzontale
            tf2::Vector3 arbitrary;
            if (std::abs(z_axis.getZ()) < 0.9){
                arbitrary = tf2::Vector3(0, 0, 1);
            } else {

                arbitrary = tf2::Vector3(0, 1, 0); // nel caso z fosse parallelo al noostro vettore usa y
            }
            // Costruisco un frame ortogonale con X, Y, Z
            tf2::Vector3 x_axis = z_axis.cross(arbitrary);
            x_axis.normalize();
            tf2::Vector3 y_axis = z_axis.cross(x_axis);
            y_axis.normalize();

            // Costruisco la rotazione (matrice)
            tf2::Matrix3x3 rotation(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z()
            );
            tf2::Quaternion q_rot;
            rotation.getRotation(q_rot); // da matrice a quaternione

            // Applica il quaternion al waypoint
            pose.orientation.x = q_rot.x();
            pose.orientation.y = q_rot.y();
            pose.orientation.z = q_rot.z();
            pose.orientation.w = q_rot.w();


            waypoints.push_back(pose); // inseriamo nel vettore posizioni la nostra posizione
        }

        // questa sezione serve solo a verificare se la prima pos è raggiungibile, se non lo è prova a ridurre la distanza dalla pianta senza sforare entrando nel box
        if (waypoints.empty()) {
            std::cerr << "Errore: waypoints vuoto\n";
        
        } else {
            move_group.setPoseTarget(waypoints[0]);
            moveit::planning_interface::MoveGroupInterface::Plan pre_plan;
            if (move_group.plan(pre_plan) == moveit::core::MoveItErrorCode::SUCCESS){
                first_is_available = true;
            } else {
                if (radius - 0.01 >= diagonal - 0.01){
                    radius -= 0.01;
                } else {
                    first_is_available = true;
                }
            }
        }

    }
    return waypoints;
}


int execute_plant_scan(std::vector<geometry_msgs::msg::Pose> waypoints, moveit::planning_interface::MoveGroupInterface& move_group){
    // forza il dobot a raggiungere la prima pos della scansione
    bool recalculate = true;

    int try_limit = 4;  //limite di tentativi per trovare la posa corretta
    int counter = 0;

    while (recalculate){
        if (waypoints.empty()) {
            std::cerr << "Errore: waypoints vuoto\n";
            return 1;
        }
        
        std::cout << counter << std::endl;
        move_group.setPoseTarget(waypoints[0]);
        moveit::planning_interface::MoveGroupInterface::Plan pre_plan;
        if (move_group.plan(pre_plan) == moveit::core::MoveItErrorCode::SUCCESS){
            move_group.execute(pre_plan);
 
            rclcpp::sleep_for(500ms);
 
            moveit_msgs::msg::RobotTrajectory trajectory; // oggetto che conterrà il movimento interpolato
            const double eef_step = 0.01;  // se ci sono 10 cm di distanza tra un punto e l'altro, con 0.01 si dice di creare a ogni cm (0.01) un punto intermedio (regola quanto fluida è la circ)
            const double jump_threshold = 0.0;  // permette o meno che faccia movimenti strani per raggiungere i punti (0.0 permette, 0.1 si ferma se troppo strano, 2.0 permissivo)
 
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // prende i waypoint e calcola un trajectory
 
            //RCLCPP_INFO(node->get_logger(), "Pianificazione cartesiana completata al %.2f%%", fraction * 100.0);
 
            if (fraction > 0.9 || counter > try_limit)  // accettiamo almeno il 90% del percorso, fraction è un numero da 0 a 1 che indica la percentuale di successo delle traiettorie
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group.execute(plan);
                recalculate = false;
            }
            else
            {
               counter ++;
               // RCLCPP_WARN(node->get_logger(), "Il piano cartesiano non è stato completato completamente.");
            }
            
        } 
     }
    return 0;
}
*/









#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>  
#include <moveit_msgs/msg/collision_object.hpp>                         
#include <shape_msgs/msg/solid_primitive.hpp>                           
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>
#include <set>

// Include custom messages
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/circumference.hpp>
#include <custom_messages/msg/optimal_point.hpp>

using namespace std::chrono_literals;

// Camera field of view parameters
const double CAMERA_FOV_HORIZONTAL = 0.3; // 10cm horizontal coverage
const double CAMERA_FOV_VERTICAL = 0.3;   // 10cm vertical coverage

struct ScanPoint {
    custom_messages::msg::OptimalPoint point;
    int index;
    bool is_boundary;
    bool covered;  // Changed from visited to covered (by camera FOV)
};

// Global variables
bool first_scan = true; // Flag to indicate if this is the first scan
custom_messages::msg::Map current_map_;
rclcpp::Logger logger_ = rclcpp::get_logger("move_cr5_node");

// Function declarations
void mapCallback(const custom_messages::msg::Map::SharedPtr msg,moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::PlanningSceneInterface planning_scene_interface);
void addObstaclesToScene(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::PlanningSceneInterface planning_scene_interface);
void processTargetPlants(moveit::planning_interface::MoveGroupInterface& move_group);
void processCircumference(geometry_msgs::msg::Point& target_to_watch,
                         const custom_messages::msg::Circumference& circumference,
                         moveit::planning_interface::MoveGroupInterface& move_group);
void identifyBoundaryPoints(std::vector<ScanPoint>& scan_points);
double calculateDistance(const custom_messages::msg::OptimalPoint& p1, 
                        const custom_messages::msg::OptimalPoint& p2);
bool attemptToReachPoint(ScanPoint& scan_point,
                        std::vector<ScanPoint>& all_points,
                        geometry_msgs::msg::Point& target_to_watch,
                        moveit::planning_interface::MoveGroupInterface& move_group);
geometry_msgs::msg::Pose createPosePointingToCenter(
    const custom_messages::msg::OptimalPoint& point,
    const geometry_msgs::msg::Point& center);
int markPointsAsCovered(ScanPoint& current_point, std::vector<ScanPoint>& all_points);
void scanUncoveredPoints(std::vector<ScanPoint>& scan_points,
                        geometry_msgs::msg::Point& target_to_watch,
                        moveit::planning_interface::MoveGroupInterface& move_group);

// Function implementations
void mapCallback(const custom_messages::msg::Map::SharedPtr msg, moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::PlanningSceneInterface planning_scene_interface)
{
    RCLCPP_INFO(logger_, " ");
    RCLCPP_INFO(logger_, "=========================================================");
    RCLCPP_INFO(logger_, "                NEW MAP RECEIVED                        ");
    RCLCPP_INFO(logger_, "=========================================================");
    RCLCPP_INFO(logger_, "[MAP] Received map from 'reworked_map'");
    RCLCPP_INFO(logger_, "[MAP] Total objects in map: %zu", msg->objects.size());
    
    current_map_ = *msg;
    
    // Count target plants
    int target_count = 0;
    int obstacle_count = 0;
    for (const auto& obj : current_map_.objects)
    {
        if (obj.target) 
            target_count++;
        else 
            obstacle_count++;
    }
    RCLCPP_INFO(logger_, "[ANALYSIS] Target plants to scan: %d", target_count);
    RCLCPP_INFO(logger_, "[ANALYSIS] Obstacles detected: %d", obstacle_count);
    
    if (target_count == 0) {
        RCLCPP_WARN(logger_, "[WARNING] No target plants found in the map!");
        return;
    }
    
    // Add obstacles to planning scene
    RCLCPP_INFO(logger_, " ");
    RCLCPP_INFO(logger_, "---------------------------------------------------------");
    RCLCPP_INFO(logger_, "[SETUP] Starting planning environment configuration");
    addObstaclesToScene(move_group,planning_scene_interface);
    
    // Process each target plant
    RCLCPP_INFO(logger_, "[EXECUTION] Starting target plant processing");
    RCLCPP_INFO(logger_, "---------------------------------------------------------");
    processTargetPlants(move_group);
    
    RCLCPP_INFO(logger_, " ");
    RCLCPP_INFO(logger_, "=========================================================");
    RCLCPP_INFO(logger_, "        ✓ MAP PROCESSING COMPLETED                      ");
    RCLCPP_INFO(logger_, "=========================================================");
}

void addObstaclesToScene(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::PlanningSceneInterface planning_scene_interface)
{
    RCLCPP_INFO(logger_, "[OBSTACLES] Adding obstacles to planning scene...");
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // Add workspace boundaries as collision object
    RCLCPP_INFO(logger_, "[OBSTACLES] > Creating workspace floor...");
    moveit_msgs::msg::CollisionObject workspace;
    workspace.header.frame_id = move_group.getPlanningFrame();
    workspace.id = "workspace_boundaries";
    
    // Add floor
    shape_msgs::msg::SolidPrimitive floor_primitive;
    floor_primitive.type = floor_primitive.BOX;
    floor_primitive.dimensions = {2.0, 2.0, 0.01};
    
    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.x = 0;
    floor_pose.position.y = 0;
    floor_pose.position.z = -0.005;    //current_map_.work_space.low_left.z - 0.005;
    floor_pose.orientation.w = 1.0;
    
    workspace.primitives.push_back(floor_primitive);
    workspace.primitive_poses.push_back(floor_pose);
    workspace.operation = workspace.ADD;
    collision_objects.push_back(workspace);
    
    RCLCPP_INFO(logger_, "[OBSTACLES] > ✓ Floor added at z=%.3f", floor_pose.position.z);
    
    // Add all objects from map as collision objects
    RCLCPP_INFO(logger_, "[OBSTACLES] > Adding objects from map...");
    for (size_t i = 0; i < current_map_.objects.size(); ++i)
    {
        const auto& object = current_map_.objects[i];
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.header.frame_id = move_group.getPlanningFrame();
        collision_obj.id = "object_" + std::to_string(i);
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        
        double size_x = object.shape.top_right.x - object.shape.low_left.x + 0.01; // Add small margin
        double size_y = object.shape.top_right.y - object.shape.low_left.y + 0.01; // Add small margin
        double size_z = object.shape.top_right.z - object.shape.low_left.z + 0.01; // Add small margin
        primitive.dimensions = {size_x, size_y, size_z};
        
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = object.shape.low_left.x + size_x / 2.0;
        box_pose.position.y = object.shape.low_left.y + size_y / 2.0;
        box_pose.position.z = object.shape.low_left.z + size_z / 2.0;
        box_pose.orientation.w = 1.0;
        
        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(box_pose);
        collision_obj.operation = collision_obj.ADD;
        collision_objects.push_back(collision_obj);
        
        std::string obj_type = object.target ? "TARGET" : "OBSTACLE";
        RCLCPP_INFO(logger_, "[OBSTACLES] > ✓ %s_%zu: size=(%.3fx%.3fx%.3f) center=(%.3f,%.3f,%.3f)",
                   obj_type.c_str(), i, size_x, size_y, size_z,
                   box_pose.position.x, box_pose.position.y, box_pose.position.z);
    }
    
    planning_scene_interface.applyCollisionObjects(collision_objects);
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(logger_, "[OBSTACLES] ✓ Added %zu collision objects to scene", collision_objects.size());
}

void processTargetPlants(moveit::planning_interface::MoveGroupInterface& move_group)
{
    int plant_number = 0;
    for (const auto& object : current_map_.objects)
    {
        if (object.target)
        {
            plant_number++;
            RCLCPP_INFO(logger_, " ");
            RCLCPP_INFO(logger_, "█████████████████████████████████████████████████████████");
            RCLCPP_INFO(logger_, "          PROCESSING TARGET PLANT #%d", plant_number);
            RCLCPP_INFO(logger_, "█████████████████████████████████████████████████████████");
            
            RCLCPP_INFO(logger_, "[PLANT-%d] Bounding box information:", plant_number);
            RCLCPP_INFO(logger_, "[PLANT-%d]   Bottom-left corner: (%.3f, %.3f, %.3f)", plant_number,
                       object.shape.low_left.x, object.shape.low_left.y, object.shape.low_left.z);
            RCLCPP_INFO(logger_, "[PLANT-%d]   Top-right corner: (%.3f, %.3f, %.3f)", plant_number,
                       object.shape.top_right.x, object.shape.top_right.y, object.shape.top_right.z);
            
            double volume_x = object.shape.top_right.x - object.shape.low_left.x;
            double volume_y = object.shape.top_right.y - object.shape.low_left.y;
            double volume_z = object.shape.top_right.z - object.shape.low_left.z;
            RCLCPP_INFO(logger_, "[PLANT-%d]   Dimensions: %.3f x %.3f x %.3f m", plant_number, volume_x, volume_y, volume_z);
            
            RCLCPP_INFO(logger_, "[PLANT-%d] Available trajectories: %zu", plant_number,
                       object.possible_trajectories.size());
            
            if (object.possible_trajectories.empty()) {
                RCLCPP_WARN(logger_, "[PLANT-%d] ⚠️  WARNING: No trajectories available!", plant_number);
                continue;
            }
            
            // Process each circumference for this plant
            for (size_t traj_idx = 0; traj_idx < object.possible_trajectories.size(); ++traj_idx)
            {
                RCLCPP_INFO(logger_, " ");
                RCLCPP_INFO(logger_, "[PLANT-%d] ═══ TRAJECTORY %zu/%zu ═══", plant_number,
                           traj_idx + 1, object.possible_trajectories.size());

                // Calculate plant center
                geometry_msgs::msg::Point target_to_watch;
                target_to_watch.x = (object.shape.low_left.x + object.shape.top_right.x) / 2.0;
                target_to_watch.y = (object.shape.low_left.y + object.shape.top_right.y) / 2.0;

                if (first_scan)
                {
                    target_to_watch.z = (object.shape.low_left.z + object.shape.top_right.z) / 2.0;
                    first_scan = false;
                    RCLCPP_INFO(logger_, "[PLANT-%d] Calculated center (first scan): (%.3f, %.3f, %.3f)", plant_number,
                               target_to_watch.x, target_to_watch.y, target_to_watch.z);
                }
                else
                {
                    target_to_watch.z = object.possible_trajectories[traj_idx].circumference[0].point.z; // Use z of first point for setting height
                    RCLCPP_INFO(logger_, "[PLANT-%d] Used center (subsequent scans): (%.3f, %.3f, %.3f)", plant_number,
                               target_to_watch.x, target_to_watch.y, target_to_watch.z);
                }
                processCircumference(target_to_watch, object.possible_trajectories[traj_idx], move_group);
            }
            first_scan = true; // Reset for next plant
            RCLCPP_INFO(logger_, " ");
            RCLCPP_INFO(logger_, "█████████████████████████████████████████████████████████");
            RCLCPP_INFO(logger_, "        ✓ PLANT #%d COMPLETED SUCCESSFULLY", plant_number);
            RCLCPP_INFO(logger_, "█████████████████████████████████████████████████████████");
        }
    }
}

void processCircumference(geometry_msgs::msg::Point& target_to_watch, 
                         const custom_messages::msg::Circumference& circumference,
                         moveit::planning_interface::MoveGroupInterface& move_group)
{
    RCLCPP_INFO(logger_, "[SCANNING] Analyzing circumference with %zu points", circumference.circumference.size());
    
    // Convert points to ScanPoint structure
    std::vector<ScanPoint> scan_points;
    int optimal_count = 0;
    int unreachable_count = 0;
    int partial_count = 0;

    // Populate scan_points and count optimal/unreachable points
    for (size_t i = 0; i < circumference.circumference.size(); ++i)
    {
        ScanPoint sp;
        sp.point = circumference.circumference[i];
        sp.index = i;
        sp.is_boundary = false;
        sp.covered = false;
        scan_points.push_back(sp);
        
        if (sp.point.optimality == 1.0) 
            optimal_count++;
        else if (sp.point.optimality == 0.0) 
            unreachable_count++;
        else 
            partial_count++;
    }
    
    RCLCPP_INFO(logger_, "[POINT-ANALYSIS] Circumference statistics:");
    RCLCPP_INFO(logger_, "[POINT-ANALYSIS]   - Optimal points (reachable): %d", optimal_count);
    RCLCPP_INFO(logger_, "[POINT-ANALYSIS]   - Unreachable points: %d", unreachable_count);
    if (partial_count > 0) {
        RCLCPP_INFO(logger_, "[POINT-ANALYSIS]   - Partial points: %d", partial_count);
    }
    
    // Identify boundary points (optimality 1 next to optimality 0)
    RCLCPP_INFO(logger_, "[POINT-ANALYSIS] Identifying boundary points...");
    identifyBoundaryPoints(scan_points);
    
    // Count boundary points
    int boundary_count = 0;
    for (const auto& sp : scan_points)
    {
        if (sp.is_boundary && sp.point.optimality == 1.0)
        {
            boundary_count++;
        }
    }
    
    if (boundary_count > 0) {
        RCLCPP_INFO(logger_, "[POINT-ANALYSIS] ✓ Found %d critical boundary points", boundary_count);
    } else {
        RCLCPP_INFO(logger_, "[POINT-ANALYSIS] No boundary points identified");
    }
    
    // Phase 1: Try to visit boundary points
    if (boundary_count > 0)
    {
        RCLCPP_INFO(logger_, " ");
        RCLCPP_INFO(logger_, "[PHASE-1] ▶ Attempting to reach boundary points");
        RCLCPP_INFO(logger_, "[PHASE-1] Objective: visit %d critical points", boundary_count);
        int visited_boundaries = 0;
        for (size_t i = 0; i < scan_points.size(); ++i)
        {
            if (scan_points[i].is_boundary && 
                scan_points[i].point.optimality == 1.0 && 
                !scan_points[i].covered)
            {
                RCLCPP_INFO(logger_, "[PHASE-1] > Attempting boundary point #%zu: (%.3f, %.3f, %.3f)", 
                           i, scan_points[i].point.point.x, scan_points[i].point.point.y, scan_points[i].point.point.z);
                
                if (attemptToReachPoint(scan_points[i], scan_points, target_to_watch, move_group))
                {
                    visited_boundaries++;
                }
            }
        }
        RCLCPP_INFO(logger_, "[PHASE-1] ✓ Completed - visited %d/%d boundary points", 
                   visited_boundaries, boundary_count);
    }
    else
    {
        RCLCPP_INFO(logger_, "[PHASE-1] ⚠ Skipped - no boundary points found");
    }
    
    // Phase 2: Visit any remaining uncovered optimal points
    RCLCPP_INFO(logger_, " ");
    RCLCPP_INFO(logger_, "[PHASE-2] ▶ Scanning remaining uncovered points");
    scanUncoveredPoints(scan_points, target_to_watch, move_group);
}

void identifyBoundaryPoints(std::vector<ScanPoint>& scan_points)
{
    size_t n = scan_points.size();
    for (size_t i = 0; i < n; ++i)
    {
        if (scan_points[i].point.optimality == 1.0)
        {
            // Check previous and next points (circular)
            size_t prev = (i - 1 + n) % n;
            size_t next = (i + 1) % n;
            
            if (scan_points[prev].point.optimality == 0.0 || 
                scan_points[next].point.optimality == 0.0)
            {
                scan_points[i].is_boundary = true;
            }
        }
    }
}

double calculateDistance(const custom_messages::msg::OptimalPoint& p1, 
                       const custom_messages::msg::OptimalPoint& p2)
{
    double dx = p1.point.x - p2.point.x;
    double dy = p1.point.y - p2.point.y;
    double dz = p1.point.z - p2.point.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool attemptToReachPoint(ScanPoint& scan_point,
                        std::vector<ScanPoint>& all_points,
                        geometry_msgs::msg::Point& target_to_watch,
                        moveit::planning_interface::MoveGroupInterface& move_group)
{
    if (scan_point.covered || scan_point.point.optimality != 1.0)
    {
        return false;
    }

    RCLCPP_DEBUG(logger_, "[MOVEMENT] Target plant center: (%.3f, %.3f, %.3f)", 
                target_to_watch.x, target_to_watch.y, target_to_watch.z);
    
    // Create pose for the scan point
    geometry_msgs::msg::Pose target_pose = createPosePointingToCenter(
        scan_point.point, target_to_watch);
    
    // Try to plan and execute movement
    RCLCPP_INFO(logger_, "[MOVEMENT] >> Planning movement to point %d...", scan_point.index);
    move_group.setPoseTarget(target_pose);

    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));
    
    if (success)
    {
        RCLCPP_INFO(logger_, "[MOVEMENT] >> ✓ Planning successful for point %d", scan_point.index);
        
        // Execute the plan
        RCLCPP_INFO(logger_, "[MOVEMENT] >> Executing movement...");
        moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
        
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger_, "[MOVEMENT] >> ✓ Movement completed successfully!");
            
            // Mark this point and nearby points as covered based on camera FOV
            int covered_count = markPointsAsCovered(scan_point, all_points);
            RCLCPP_INFO(logger_, "[COVERAGE] >> ✓ Marked %d points as covered by camera FOV", covered_count);

            RCLCPP_INFO(logger_, "[SCANNING] >> Waiting for scanning (11 seconds)...");
            rclcpp::sleep_for(11000ms); // Allow time for scanning
            return true;
        }
        else
        {
            RCLCPP_ERROR(logger_, "[MOVEMENT] >> ✗ Execution ERROR - code: %d", 
                        execution_result.val);
        }
    }
    else
    {
        RCLCPP_WARN(logger_, "[MOVEMENT] >> ✗ Planning failed for point %d", scan_point.index);
        
    }
    
    return false;
}

geometry_msgs::msg::Pose createPosePointingToCenter(
    const custom_messages::msg::OptimalPoint& point,
    const geometry_msgs::msg::Point& center)
{
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
    if (std::abs(z_axis.getZ()) > 0.9)
    {
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

int markPointsAsCovered(ScanPoint& current_point, std::vector<ScanPoint>& all_points)
{
    int covered_count = 1; // Current point itself
    current_point.covered = true;
    
    // Mark nearby points as covered based on camera FOV
    for (auto& point : all_points)
    {
        if (!point.covered && point.point.optimality == 1.0)
        {
            double dist = calculateDistance(current_point.point, point.point);
            
            // Simple FOV model: if point is within FOV range, mark as covered
            if (dist < CAMERA_FOV_HORIZONTAL)
            {
                point.covered = true;
                covered_count++;
                RCLCPP_DEBUG(logger_, "        Covered point %d at distance %.3f", 
                            point.index, dist);
            }
        }
    }
    
    return covered_count;
}

void scanUncoveredPoints(std::vector<ScanPoint>& scan_points,
                        geometry_msgs::msg::Point& target_to_watch,
                        moveit::planning_interface::MoveGroupInterface& move_group)
{
    // Find all uncovered optimal points
    std::vector<size_t> uncovered_indices;
    for (size_t i = 0; i < scan_points.size(); ++i)
    {
        if (scan_points[i].point.optimality == 1.0 && !scan_points[i].covered)
        {
            uncovered_indices.push_back(i);
        }
    }
    
    RCLCPP_INFO(logger_, "[PHASE-2] Found %zu uncovered points to scan", uncovered_indices.size());
    
    if (uncovered_indices.empty())
    {
        RCLCPP_INFO(logger_, "[PHASE-2] ✓ All optimal points are already covered!");
        return;
    }
    
    // Process uncovered points
    RCLCPP_INFO(logger_, "[PHASE-2] Starting scan of remaining points...");
    int successful_scans = 0;
    int failed_scans = 0;
    
    for (size_t idx : uncovered_indices)
    {
        if (!scan_points[idx].covered) // Double check as it might have been covered by previous scan
        {
            RCLCPP_INFO(logger_, "[PHASE-2] > Attempting uncovered point #%zu: (%.3f, %.3f, %.3f)", 
                       idx, scan_points[idx].point.point.x, scan_points[idx].point.point.y, scan_points[idx].point.point.z);

            if (attemptToReachPoint(scan_points[idx], scan_points, target_to_watch, move_group))
            {
                successful_scans++;
                RCLCPP_INFO(logger_, "[PHASE-2] > ✓ Success for point #%zu", idx);
            }
            else
            {
                failed_scans++;
                RCLCPP_WARN(logger_, "[PHASE-2] > ✗ Failure for point #%zu", idx);
            }
        }
    }
    
    RCLCPP_INFO(logger_, "[PHASE-2] ✓ Completed - %d successes, %d failures", 
               successful_scans, failed_scans);
    
    // Log final coverage statistics
    int total_optimal = 0;
    int covered = 0;
    for (const auto& sp : scan_points)
    {
        if (sp.point.optimality == 1.0)
        {
            total_optimal++;
            if (sp.covered)
            {
                covered++;
            }
        }
    }
    
    double coverage_percentage = total_optimal > 0 ? (100.0 * covered / total_optimal) : 0.0;
    RCLCPP_INFO(logger_, " ");
    RCLCPP_INFO(logger_, "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓");
    RCLCPP_INFO(logger_, "            FINAL COVERAGE REPORT");
    RCLCPP_INFO(logger_, "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓");
    RCLCPP_INFO(logger_, "[RESULT] Points covered: %d/%d (%.1f%%)",
                covered, total_optimal, coverage_percentage);
    
    // Log any remaining uncovered points
    if (covered < total_optimal)
    {
        RCLCPP_WARN(logger_, "[WARNING] %d points remain UNCOVERED:", total_optimal - covered);
        for (const auto& sp : scan_points)
        {
            if (sp.point.optimality == 1.0 && !sp.covered)
            {
                RCLCPP_WARN(logger_, "[UNCOVERED]   - Point #%d: (%.3f, %.3f, %.3f)", 
                           sp.index, sp.point.point.x, sp.point.point.y, sp.point.point.z);
            }
        }
    }
    else
    {
        RCLCPP_INFO(logger_, "[RESULT] ✓ COMPLETE COVERAGE - All points have been visited!");
    }
    RCLCPP_INFO(logger_, "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("cr5_moveit_cpp_planner");
    
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
    
    // QoS configuration for subscriber (latching)
    RCLCPP_INFO(node->get_logger(), "[PHASE 3] Configuring map subscriber...");
    rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
    sub_qos.transient_local();
    
    // Subscribe to reworked_map topic
    auto map_subscriber = node->create_subscription<custom_messages::msg::Map>(
        "reworked_map",
        sub_qos,
        [&](const custom_messages::msg::Map::SharedPtr msg) {
            mapCallback(msg, move_group, planning_scene_interface);
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



/*

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cr5_moveit_cpp_planner");

    // Inizializza MoveGroupInterface per il planning group del CR5
    moveit::planning_interface::MoveGroupInterface move_group(node, "cr5_group");


    // Configura parametri di esecuzione
    move_group.setPoseReferenceFrame("base_link");
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);


    // Definisce la posizione target
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.596;
    target_pose.position.y = 0.439;
    target_pose.position.z = 0.725;

    // Orientamento neutro
    target_pose.orientation.w = 1.0;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;

    move_group.setPoseTarget(target_pose);

    // Pianifica
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Piano trovato! Eseguo...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Pianificazione fallita.");
    }

    rclcpp::shutdown();
    return 0;
}

*/