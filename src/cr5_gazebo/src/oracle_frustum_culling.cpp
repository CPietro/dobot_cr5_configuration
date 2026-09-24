#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/object.hpp>
#include <custom_messages/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <map>

namespace cr5_gazebo
{

class OracleFrustumCulling : public rclcpp::Node
{
public:
  explicit OracleFrustumCulling(const rclcpp::NodeOptions & options)
  : Node("oracle_frustum_culling", options)
  {
    // Parameters initialization
    this->declare_parameter<std::string>("global_frame", "world");
    this->declare_parameter<std::string>("camera_frame", "zed_optical_link");
    
    this->declare_parameter<double>("fov_horizontal_deg", 86.0);
    this->declare_parameter<double>("fov_vertical_deg", 70.0);
    this->declare_parameter<double>("min_depth", 0.1);
    this->declare_parameter<double>("max_depth", 8.0);
    
    this->declare_parameter<std::vector<std::string>>("target_keywords", {"pianta", "plant", "target"});
    this->declare_parameter<std::vector<std::string>>("obstacle_keywords", {"ostacolo", "obstacle", "wall", "box"});
    
    this->declare_parameter<double>("dummy_size_x", 0.10);
    this->declare_parameter<double>("dummy_size_y", 0.10);
    this->declare_parameter<double>("dummy_size_z", 0.20);

    // Workspace limits
    this->declare_parameter<double>("ws_x_min", -1.0);
    this->declare_parameter<double>("ws_y_min", -1.0);
    this->declare_parameter<double>("ws_z_min", -1.0);
    this->declare_parameter<double>("ws_x_max", 1.0);
    this->declare_parameter<double>("ws_y_max", 1.0);
    this->declare_parameter<double>("ws_z_max", 1.0);

    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher for /Boing with Transient Local QoS (latching)
    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.transient_local();
    map_pub_ = this->create_publisher<custom_messages::msg::Map>("/Boing", pub_qos);

    // Subscriber for Gazebo Model States
    model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", 10,
      std::bind(&OracleFrustumCulling::modelStatesCallback, this, std::placeholders::_1)
    );

    bbox_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/gazebo/model_bboxes", 10,
      std::bind(&OracleFrustumCulling::bboxCallback, this, std::placeholders::_1)
    );

    last_publish_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Oracle Frustum Culling Node started.");
  }

private:
  void bboxCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (const auto& marker : msg->markers) {
      if (marker.points.size() >= 2) {
        model_bboxes_[marker.ns] = std::make_pair(marker.points[0], marker.points[1]);
      }
    }
  }

  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (map_pub_->get_subscription_count() == 0) {
      return;
    }

    // Throttle frequency to 10 Hz 
    rclcpp::Time current_time = this->now();
    if ((current_time - last_publish_time_).seconds() < 0.1) {
      return;
    }
    last_publish_time_ = current_time;

    // Dynamically find the robot's spawn offset to align Gazebo World with MoveIt World
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "dobot_cr5") {
        offset_x = msg->pose[i].position.x;
        offset_y = msg->pose[i].position.y;
        offset_z = msg->pose[i].position.z;
        break;
      }
    }

    std::string global_frame = this->get_parameter("global_frame").as_string();
    std::string camera_frame = this->get_parameter("camera_frame").as_string();

    geometry_msgs::msg::TransformStamped t;
    try {
      // Lookup the latest transform from global to camera
      t = tf_buffer_->lookupTransform(camera_frame, global_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Could not transform %s to %s: %s", global_frame.c_str(), camera_frame.c_str(), ex.what());
      return;
    }

    custom_messages::msg::Map out_map;

    // The workspace limits in absolute world coordinates dynamically shift with the robot
    out_map.work_space.low_left.x = this->get_parameter("ws_x_min").as_double() + offset_x;
    out_map.work_space.low_left.y = this->get_parameter("ws_y_min").as_double() + offset_y;
    out_map.work_space.low_left.z = this->get_parameter("ws_z_min").as_double() + offset_z;
    out_map.work_space.top_right.x = this->get_parameter("ws_x_max").as_double() + offset_x;
    out_map.work_space.top_right.y = this->get_parameter("ws_y_max").as_double() + offset_y;
    out_map.work_space.top_right.z = this->get_parameter("ws_z_max").as_double() + offset_z;

    auto target_kw = this->get_parameter("target_keywords").as_string_array();
    auto obstacle_kw = this->get_parameter("obstacle_keywords").as_string_array();

    double fov_h = this->get_parameter("fov_horizontal_deg").as_double() * M_PI / 180.0;
    double fov_v = this->get_parameter("fov_vertical_deg").as_double() * M_PI / 180.0;
    double min_z = this->get_parameter("min_depth").as_double();
    double max_z = this->get_parameter("max_depth").as_double();

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& model_name = msg->name[i];
      
      bool is_target = false;
      bool is_obstacle = false;

      for (const auto& kw : target_kw) {
        if (model_name.find(kw) != std::string::npos) {
          is_target = true;
          break;
        }
      }

      if (!is_target) {
        for (const auto& kw : obstacle_kw) {
          if (model_name.find(kw) != std::string::npos) {
            is_obstacle = true;
            break;
          }
        }
      }

      if (!is_target && !is_obstacle) continue;

      custom_messages::msg::Point low_left;
      custom_messages::msg::Point top_right;
      geometry_msgs::msg::Point global_pos = msg->pose[i].position;

      // Use real absolute Bounding Boxes from Gazebo plugin if available
      if (model_bboxes_.find(model_name) != model_bboxes_.end()) {
        low_left.x = model_bboxes_[model_name].first.x - offset_x;
        low_left.y = model_bboxes_[model_name].first.y - offset_y;
        low_left.z = model_bboxes_[model_name].first.z - offset_z;

        top_right.x = model_bboxes_[model_name].second.x - offset_x;
        top_right.y = model_bboxes_[model_name].second.y - offset_y;
        top_right.z = model_bboxes_[model_name].second.z - offset_z;
        
        // Compute center for frustum calculation
        global_pos.x = (low_left.x + top_right.x) / 2.0;
        global_pos.y = (low_left.y + top_right.y) / 2.0;
        global_pos.z = (low_left.z + top_right.z) / 2.0;
      } else {
        // Fallback to dummy sizes if plugin data is missing
        double dim_x = this->get_parameter("dummy_size_x").as_double();
        double dim_y = this->get_parameter("dummy_size_y").as_double();
        double dim_z = this->get_parameter("dummy_size_z").as_double();
        
        // Apply offset to fallback global_pos
        global_pos.x -= offset_x;
        global_pos.y -= offset_y;
        global_pos.z -= offset_z;

        low_left.x = global_pos.x - (dim_x / 2.0);
        low_left.y = global_pos.y - (dim_y / 2.0);
        low_left.z = global_pos.z;

        top_right.x = global_pos.x + (dim_x / 2.0);
        top_right.y = global_pos.y + (dim_y / 2.0);
        top_right.z = global_pos.z + dim_z;
      }

      geometry_msgs::msg::PointStamped pt_global;
      pt_global.header.frame_id = global_frame;
      pt_global.header.stamp = this->get_clock()->now();
      pt_global.point = global_pos;

      geometry_msgs::msg::PointStamped pt_camera;
      try {
        tf2::doTransform(pt_global, pt_camera, t);
      } catch (const tf2::TransformException & ex) {
        continue;
      }

      // Frustum Culling check
      double cx = pt_camera.point.x;
      double cy = pt_camera.point.y;
      double cz = pt_camera.point.z;

      if (cz < min_z || cz > max_z) continue;

      double max_x_at_z = cz * std::tan(fov_h / 2.0);
      double max_y_at_z = cz * std::tan(fov_v / 2.0);

      if (std::abs(cx) > max_x_at_z || std::abs(cy) > max_y_at_z) continue;

      // Object passed culling, create bounding box in GLOBAL coordinates
      custom_messages::msg::Object obj;
      obj.target = is_target;
      
      // Bottom left corner (min)
      obj.shape.low_left = low_left;

      // Top right corner (max)
      obj.shape.top_right = top_right;

      out_map.objects.push_back(obj);
    }

    map_pub_->publish(out_map);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<custom_messages::msg::Map>::SharedPtr map_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_sub_;
  std::map<std::string, std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> model_bboxes_;
  rclcpp::Time last_publish_time_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(cr5_gazebo::OracleFrustumCulling)
