#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <custom_messages/msg/optimal_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/circumference.hpp>

custom_messages::msg::Map map;

class ZedMapPublisherNode : public rclcpp::Node
{
public:
    ZedMapPublisherNode()
    : Node("zed_map_publisher_node")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();

        map_publisher_ = this->create_publisher<custom_messages::msg::Map>("Boing", qos);

    }

    rclcpp::Publisher<custom_messages::msg::Map>::SharedPtr map_publisher_;

    void publishMap()
    {
        map_publisher_->publish(map);
        RCLCPP_INFO(this->get_logger(), "Mappa iniziale pubblicata su Boing (lacciata).");
    }
};

void create_map(){
    geometry_msgs::msg::Point ws_low_left;
    geometry_msgs::msg::Point ws_top_right;

    ws_low_left.x = -1;  // Quadrato 180cm x 180cm centrato sul robot
    ws_low_left.y = -1;
    ws_low_left.z = -1;

    ws_top_right.x = 1;
    ws_top_right.y = 1;
    ws_top_right.z = 1;

    custom_messages::msg::BoundingBox ws;

    ws.low_left = ws_low_left;
    ws.top_right = ws_top_right;

    map.work_space = ws;

    custom_messages::msg::Object plant1;
    geometry_msgs::msg::Point p1_low_left;
    geometry_msgs::msg::Point p1_top_right;

    // Prima pianta: davanti al robot, spostata a sinistra di 15cm
    // Centro rettangolo frontale Y = 0.45, pianta centrata a X = -0.15
    // Dimensioni: 10x10x10 cm = 0.1x0.1x0.1 m
    p1_low_left.x = -0.30;  // Centro X -0.15 - metà larghezza 0.05
    p1_low_left.y = 0.40;   // Centro Y fisso a 0.45 - metà profondità 0.05
    p1_low_left.z = 0.0;    // Poggia per terra

    p1_top_right.x = -0.10; // Centro X -0.15 + metà larghezza 0.05
    p1_top_right.y = 0.50;  // Centro Y fisso a 0.45 + metà profondità 0.05
    p1_top_right.z = 0.20;   // Altezza 30cm

    custom_messages::msg::BoundingBox shape1;

    shape1.low_left  = p1_low_left;
    shape1.top_right = p1_top_right;

    plant1.target = true;
    plant1.shape = shape1;

    map.objects.push_back(plant1);



    custom_messages::msg::Object plant2;
    geometry_msgs::msg::Point p2_low_left;
    geometry_msgs::msg::Point p2_top_right;

    // Seconda pianta: davanti al robot, spostata a destra di 15cm
    // Centro rettangolo frontale Y = 0.45, pianta centrata a X = +0.15
    p2_low_left.x = 0.05;   // Centro X +0.15 - metà larghezza 0.05
    p2_low_left.y = 0.40;   // Centro Y fisso a 0.45 - metà profondità 0.05
    p2_low_left.z = 0.0;    // Poggia per terra

    p2_top_right.x = 0.15;  // Centro X +0.15 + metà larghezza 0.05
    p2_top_right.y = 0.65;  // Centro Y fisso a 0.45 + metà profondità 0.05
    p2_top_right.z = 0.20;   // Altezza 30cm

    custom_messages::msg::BoundingBox shape2;

    shape2.low_left  = p2_low_left;
    shape2.top_right = p2_top_right;

    plant2.target = true;
    plant2.shape = shape2;

    map.objects.push_back(plant2);


    custom_messages::msg::Object obstacle1;
    geometry_msgs::msg::Point o1_low_left;
    geometry_msgs::msg::Point o1_top_right;

    o1_low_left.x = -0.5;
    o1_low_left.y = -0.5;
    o1_low_left.z = -0.5;

    o1_top_right.x = -0.4;
    o1_top_right.y = -0.4;
    o1_top_right.z = -0.4;

    custom_messages::msg::BoundingBox shape3;

    shape3.low_left  = o1_low_left;
    shape3.top_right = o1_top_right;

    obstacle1.target = false;
    obstacle1.shape = shape3;

    //map.objects.push_back(obstacle1);
    
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto zed_map_publisher_node = std::make_shared<ZedMapPublisherNode>();

    create_map();

    // Chiama il metodo per pubblicare la mappa dal main
    zed_map_publisher_node->publishMap();

    rclcpp::spin(zed_map_publisher_node); // Mantieni il nodo in esecuzione (anche se brevemente) per la pubblicazione
    rclcpp::shutdown();
    return 0;
}