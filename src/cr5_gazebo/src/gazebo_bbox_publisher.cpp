#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
    update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboBBoxPublisher::OnUpdate, this));
    last_publish_time_ = world_->SimTime();
    RCLCPP_INFO(ros_node_->get_logger(), "Gazebo BBox Publisher Plugin loaded (Pure World AABB).");
  }

  void OnUpdate()
  {
    common::Time current_time = world_->SimTime();
    if ((current_time - last_publish_time_).Double() < 0.1) return;
    last_publish_time_ = current_time;

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
  event::ConnectionPtr update_connection_;
  common::Time last_publish_time_;
};

GZ_REGISTER_WORLD_PLUGIN(GazeboBBoxPublisher)
}
