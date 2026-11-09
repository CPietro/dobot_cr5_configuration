#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class GazeboBBoxPublisher : public WorldPlugin
{
public:
  GazeboBBoxPublisher() : WorldPlugin() {}

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    world_ = _world;
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    pub_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>("/gazebo/model_bboxes", 10);
    joint_state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboBBoxPublisher::OnUpdate, this));
    last_publish_time_ = world_->SimTime();
    max_sim_time_ = world_->SimTime();
    RCLCPP_INFO(ros_node_->get_logger(), "Gazebo BBox Publisher Plugin loaded (Pure World AABB).");
  }

  void OnUpdate()
  {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_publish_time_).Double();
    if (dt >= 0.0 && dt < 0.1) return;
    last_publish_time_ = current_time;

    if (current_time > max_sim_time_) {
      max_sim_time_ = current_time;
    } else if (current_time < max_sim_time_) {
      // Workaround for upstream ROS 2 Humble gazebo_ros2_control bug:
      // GazeboRosControlPrivate::Update() stops updating controllers (including joint_state_broadcaster)
      // after a simulation time reset until SimTime catches back up to the pre-reset timestamp.
      // Publish /joint_states directly from the Gazebo robot model during this recovery window
      // so robot_state_publisher and RViz2 keep the TF tree (world -> ... -> zed_optical_link) alive.
      physics::ModelPtr robot_model = world_->ModelByName("dobot_cr5");
      if (robot_model) {
        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = ros_node_->now();
        for (const auto &joint : robot_model->GetJoints()) {
          if (joint && joint->DOF() > 0) {
            js_msg.name.push_back(joint->GetName());
            js_msg.position.push_back(joint->Position(0));
            js_msg.velocity.push_back(joint->GetVelocity(0));
          }
        }
        if (!js_msg.name.empty()) {
          joint_state_pub_->publish(js_msg);
        }
      }
    }

    visualization_msgs::msg::MarkerArray msg;
    physics::Model_V models = world_->Models();
    int id = 0;

    for (auto const &model : models) {
      if (!model) continue;
      
      // BoundingBox() returns the AABB in the WORLD coordinate frame!
      ignition::math::AxisAlignedBox world_bbox = model->BoundingBox();
      const auto &bmin = world_bbox.Min();
      const auto &bmax = world_bbox.Max();

      // Sanity check: if collision shapes are missing, broken, or uninitialized,
      // Gazebo returns garbage, infinite, NaN, or inverted values (e.g., 2e17). We ignore these.
      if (!std::isfinite(bmin.X()) || !std::isfinite(bmin.Y()) || !std::isfinite(bmin.Z()) ||
          !std::isfinite(bmax.X()) || !std::isfinite(bmax.Y()) || !std::isfinite(bmax.Z()) ||
          std::abs(bmin.X()) > 1000.0 || std::abs(bmin.Y()) > 1000.0 || std::abs(bmin.Z()) > 1000.0 ||
          std::abs(bmax.X()) > 1000.0 || std::abs(bmax.Y()) > 1000.0 || std::abs(bmax.Z()) > 1000.0 ||
          bmin.X() > bmax.X() || bmin.Y() > bmax.Y() || bmin.Z() > bmax.Z()) {
        continue;
      }
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros_node_->now();
      marker.ns = model->GetName();
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      
      geometry_msgs::msg::Point p_min, p_max;
      p_min.x = bmin.X();
      p_min.y = bmin.Y();
      p_min.z = bmin.Z();
      
      p_max.x = bmax.X();
      p_max.y = bmax.Y();
      p_max.z = bmax.Z();
      
      marker.points.push_back(p_min);
      marker.points.push_back(p_max);
      
      msg.markers.push_back(marker);
    }
    
    pub_->publish(msg);
  }

private:
  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  event::ConnectionPtr update_connection_;
  common::Time last_publish_time_;
  common::Time max_sim_time_;
};

GZ_REGISTER_WORLD_PLUGIN(GazeboBBoxPublisher)
}
