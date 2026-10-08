/*#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>

#include <custom_messages/msg/optimal_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/circumference.hpp>

// Aggiungi queste inclusioni per MoveIt! e i tipi di messaggi necessari
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp> // Già presente, ma assicuriamoci sia abilitato
#include <chrono>

using namespace std::chrono_literals;

// Rimuoviamo le variabili globali che non sono più necessarie per questo scopo
// custom_messages::msg::Map map;
// custom_messages::msg::BoundingBox shape;
// geometry_msgs::msg::Point center; // Questa non ci serve più per il calcolo del centro

// La funzione calculate_center non sarà più chiamata per questo scopo
// void calculate_center(custom_messages::msg::BoundingBox& shape);


class PublisherPose : public rclcpp::Node
{
  public:
    PublisherPose()
    : Node("pubisher_pose")
    {
       // Inizializzazione del publisher per /dobot_pose
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dobot_pose", 10);
      RCLCPP_INFO(this->get_logger(), "Publisher creato per il topic: /dobot_pose");

      // Imposta un timer che chiamerà il metodo timer_callback ogni 500 millisecondi
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PublisherPose::timer_callback, this));
      RCLCPP_INFO(this->get_logger(), "Timer impostato per pubblicare ogni 500ms.");
    }
  
  private:
  void timer_callback()
  {
    // ---------- Inizializzazione di MoveIt! e del link dell'end-effector ----------
    const std::string planning_group = "cr5_group"; // <<< Assicurati che sia il nome corretto!

    // Inizializzazione di MoveGroupInterface. Crea un nodo temporaneo per MoveGroupInterface.
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        rclcpp::Node::make_shared("move_group_interface_node_for_pose"), // Nome del nodo interno a MoveGroupInterface
        planning_group
    );

    // Imposta il nome del link dell'end-effector
    end_effector_link_name_ = move_group_->getEndEffectorLink();
    if (end_effector_link_name_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Impossibile ottenere il nome del link dell'end-effector da MoveIt!. Assicurati che un end_effector sia definito nel tuo SRDF.");
        // Fallback: DEVI IMPOSTARE IL NOME CORRETTO QUI SE AUTOMATICAMENTE NON LO TROVA
        end_effector_link_name_ = "tool0"; // Esempio: "tool0", "gripper_link", ecc.
        RCLCPP_WARN(this->get_logger(), "Impostato il link dell'end-effector di default a: %s", end_effector_link_name_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Link dell'end-effector identificato: %s", end_effector_link_name_.c_str());
    }
    // ---------- Fine Inizializzazione MoveIt! ----------

    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(end_effector_link_name_);
    publisher_->publish(current_pose);
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_; // Il publisher
  rclcpp::TimerBase::SharedPtr timer_; // Il timer per la pubblicazione periodica
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string end_effector_link_name_; // Per memorizzare il nome del link dell'end-effector
};











----------------------------------------------------------------













class TrackSubscriber : public rclcpp::Node
{
public:
  TrackSubscriber()
    : Node("track_subscriber")
  {
    // Inizializzazione del sottoscrittore per /Boing
    // Questo è ancora utile se vogliamo innescare la pubblicazione della posa corrente alla ricezione di un messaggio
    subscription_ = this->create_subscription<custom_messages::msg::Map>(
      "/Boing", 10, std::bind(&TrackSubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Sottoscritto al topic: /Boing");

    // Inizializzazione del publisher per /dobot_pose
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dobot_pose", 10);
    RCLCPP_INFO(this->get_logger(), "Publisher creato per il topic: /dobot_pose");

    // ---------- AGGIUNTE PER MOVEIT! ----------
    // Dichiarazione del nome del gruppo di pianificazione (es. "dobot_arm")
    // Devi sostituire "your_robot_arm" con il nome effettivo del tuo gruppo di pianificazione in MoveIt!
    // Puoi trovarlo nel file SRDF del tuo robot (es. <group name="your_robot_arm">)
    const std::string planning_group = "dobot_arm"; // <<<<< SOSTITUISCI QUESTO CON IL NOME DEL TUO GRUPPO DI PIANIFICAZIONE

    // Inizializzazione di MoveGroupInterface
    // Richiede un nodo ROS e il nome del gruppo di pianificazione
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        rclcpp::Node::make_shared("move_group_interface_node"), // Crea un nodo temporaneo per MoveGroupInterface
        planning_group
    );
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface inizializzata per il gruppo: %s", planning_group.c_str());

    // Imposta il nome del link dell'end-effector (es. "tool0" o il nome del tuo gripper link)
    // Questo deve corrispondere al nome del link dell'end-effector nel tuo URDF/SRDF
    end_effector_link_name_ = move_group_->getEndEffectorLink(); // Tenta di ottenerlo automaticamente
    if (end_effector_link_name_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Impossibile ottenere il nome del link dell'end-effector da MoveIt!. Assicurati che un end_effector sia definito nel tuo SRDF.");
        // Potresti voler impostare un valore di default qui se non lo trova automaticamente
        // end_effector_link_name_ = "your_default_end_effector_link"; // <<<<< POTRESTI DOVER SOSTITUIRE QUESTO
    } else {
        RCLCPP_INFO(this->get_logger(), "Link dell'end-effector identificato: %s", end_effector_link_name_.c_str());
    }

    // ---------- FINE AGGIUNTE PER MOVEIT! ----------
  }

private:
  // Funzione di callback per i messaggi ricevuti dal topic /Boing
  void topic_callback(const std::shared_ptr<custom_messages::msg::Map> msg)
  {
    RCLCPP_INFO(this->get_logger(), "Mappa ricevuta. Publico la posa attuale dell'end-effector.");

    // ---------- MODIFICHE ALLA CALLBACK ----------
    if (end_effector_link_name_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Impossibile pubblicare la posa: il nome del link dell'end-effector non è stato impostato.");
        return;
    }

    // Ottieni la posa attuale dell'end-effector
    // getCurrentPose() restituisce una PoseStamped, che è esattamente quello che vogliamo pubblicare.
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(end_effector_link_name_);

    // Pubblica la posa attuale
    publisher_->publish(current_pose);
    RCLCPP_INFO(this->get_logger(), "Pubblicata la posa attuale dell'end-effector: x:%f, y:%f, z:%f",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    // ---------- FINE MODIFICHE ALLA CALLBACK ----------
  }

  rclcpp::Subscription<custom_messages::msg::Map>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; // Variabile membro per MoveIt!
  std::string end_effector_link_name_; // Variabile membro per il nome del link dell'end-effector
};

// La funzione calculate_center non è più necessaria, può essere rimossa o ignorata.
// void calculate_center(custom_messages::msg::BoundingBox& shape){
//     geometry_msgs::msg::Point p1 = shape.low_left;
//     geometry_msgs::msg::Point p2 = shape.top_right;
//     center.x = p1.x + (p2.x - p1.x) / 2.0;
//     center.y = p1.y + (p2.y - p1.y) / 2.0;
//     center.z = p1.z + (p2.z - p1.z) / 2.0;
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Per MoveIt! è necessario che il nodo abbia un executor con un numero di thread adeguato.
  // Un MultiThreadedExecutor è spesso consigliato quando si usa MoveIt!
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TrackSubscriber>();
  executor.add_node(node);
  executor.spin(); // Usa l'executor per far girare il nodo
  rclcpp::shutdown();
  return 0;
}
*/