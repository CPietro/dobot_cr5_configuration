#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>  
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

custom_messages::msg::Map map;

class ReconnaissanceNode : public rclcpp::Node
{
public:
    ReconnaissanceNode()
    : Node("reconnaissance_node")
    {
        RCLCPP_INFO(this->get_logger(), "Reconnaissance node started.");

        // Inizializza MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "cr5_group");

        // QoS
        auto best_effort_qos = rclcpp::QoS(10).best_effort();
        auto localized_qos = rclcpp::QoS(1).reliable();

        // Subscribers
        plant_center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "plant_center", best_effort_qos,
            std::bind(&ReconnaissanceNode::plant_center_callback, this, std::placeholders::_1)
        );

        final_plant_pos_sub_ = this->create_subscription<custom_messages::msg::Object>(
            "final_plant_pos", localized_qos,
            std::bind(&ReconnaissanceNode::final_plant_pos_callback, this, std::placeholders::_1)
        );

        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("recon_pose", 10);
        localized_plant_pub_ = this->create_publisher<std_msgs::msg::Bool>("localized_plant", localized_qos);
    }

private:
    void plant_center_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received plant center: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);

        static geometry_msgs::msg::Point previous_point;
        static bool is_first_call = true;

        double distance = std::sqrt(std::pow(msg->x - previous_point.x, 2) +
                                    std::pow(msg->y - previous_point.y, 2) +
                                    std::pow(msg->z - previous_point.z, 2));

        if (is_first_call || distance > 0.01)
        {
            is_first_call = false;
            previous_point = *msg;

            // Pose attuale
            geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

            // Calcola orientamento verso il centro pianta
            geometry_msgs::msg::Pose target_pose = current_pose.pose;
            tf2::Vector3 direction(msg->x - current_pose.pose.position.x,
                                   msg->y - current_pose.pose.position.y,
                                   msg->z - current_pose.pose.position.z);
            direction.normalize();

            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, atan2(direction.y(), direction.x()));
            target_pose.orientation.x = orientation.x();
            target_pose.orientation.y = orientation.y();
            target_pose.orientation.z = orientation.z();
            target_pose.orientation.w = orientation.w();

            // Pianificazione
            move_group_->setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::core::MoveItErrorCode result = move_group_->plan(plan);
            bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
            {
                move_group_->execute(plan);
                RCLCPP_INFO(this->get_logger(), "End effector oriented towards plant center.");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to plan motion.");
            }

            // Pubblica pose
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "world";
            pose.pose = target_pose;
            pose_pub_->publish(pose);

            // Segnale pianta localizzata
            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            localized_plant_pub_->publish(done_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Movement is less than 1 cm, skipping motion planning.");
            std_msgs::msg::Bool done_msg;
            done_msg.data = false;
            localized_plant_pub_->publish(done_msg);
        }

    }

    void final_plant_pos_callback(const custom_messages::msg::Object::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received final plant position: x=%f, y=%f, z=%f", msg->position.x, msg->position.y, msg->position.z);

        // Aggiungi l'oggetto pianta all'array Objects nella variabile map
        custom_messages::msg::Object plant_object;
        plant_object = *msg; // Copia il messaggio ricevuto nell'oggetto pianta
        map.objects.push_back(plant_object);

        RCLCPP_INFO(this->get_logger(), "Plant object added to map. Total objects in map: %zu", map.objects.size());
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr plant_center_sub_;
    rclcpp::Subscription<custom_messages::msg::Object>::SharedPtr final_plant_pos_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localized_plant_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReconnaissanceNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
