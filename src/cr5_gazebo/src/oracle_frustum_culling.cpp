#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <custom_messages/msg/map.hpp>
#include <custom_messages/msg/object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_messages/msg/bounding_box.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <algorithm>
#include <limits>
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
    // NOTE on frames: two different frames are called "world" in this system.
    //  - Gazebo world: scene origin; /gazebo/model_states and /gazebo/model_bboxes use it.
    //  - TF/MoveIt world: URDF root link; it coincides with the frame of the robot MODEL in
    //    Gazebo (i.e. it sits at the robot spawn pose, e.g. (0.2, -0.6, 0) in Gazebo coords).
    // This node converts Gazebo-world boxes into the robot model frame by applying the inverse
    // of the robot model pose, then uses TF from `global_frame` to the camera for culling.
    // Hence `global_frame` MUST be the TF frame that coincides with the Gazebo robot model
    // frame (the URDF root, "world"); any other value yields a silent, wrong /Boing map.
    rcl_interfaces::msg::ParameterDescriptor global_frame_desc;
    global_frame_desc.description =
      "TF frame coinciding with the Gazebo frame of the robot model (URDF root link). "
      "Output boxes are expressed in this frame. Do NOT set it to e.g. base_link: "
      "the conversion from Gazebo world relies on this equivalence.";
    this->declare_parameter<std::string>("global_frame", "world", global_frame_desc);
    this->declare_parameter<std::string>("camera_frame", "zed_optical_link");
    this->declare_parameter<std::string>("robot_model_name", "dobot_cr5");
    
    this->declare_parameter<double>("fov_horizontal_deg", 85.94);
    this->declare_parameter<double>("fov_vertical_deg", 69.39);
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

    // Wait until GazeboBBoxPublisher has populated bounding boxes
    if (model_bboxes_.empty()) {
      return;
    }

    // Throttle frequency to 10 Hz (handle Gazebo simulation time reset when dt < 0)
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_publish_time_).seconds();
    if (dt < 0.0) {
      tf_buffer_->clear();
    } else if (dt < 0.1) {
      return;
    }
    last_publish_time_ = current_time;

    // Find the robot model pose in the Gazebo world: its inverse maps Gazebo-world points into
    // the robot model frame, which is the TF `global_frame` (see note in the constructor).
    const std::string robot_name = this->get_parameter("robot_model_name").as_string();
    tf2::Transform robot_tf_inv;
    robot_tf_inv.setIdentity();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == robot_name) {
        tf2::Transform robot_tf;
        tf2::fromMsg(msg->pose[i], robot_tf);
        robot_tf_inv = robot_tf.inverse();
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

    // Output is expressed in `global_frame` (robot model frame; with the default xacro
    // x/y/z = 0 the robot base is at its origin).
    // Workspace limits are already relative to the robot base, so no offset addition is needed.
    out_map.work_space.low_left.x = this->get_parameter("ws_x_min").as_double();
    out_map.work_space.low_left.y = this->get_parameter("ws_y_min").as_double();
    out_map.work_space.low_left.z = this->get_parameter("ws_z_min").as_double();
    out_map.work_space.top_right.x = this->get_parameter("ws_x_max").as_double();
    out_map.work_space.top_right.y = this->get_parameter("ws_y_max").as_double();
    out_map.work_space.top_right.z = this->get_parameter("ws_z_max").as_double();

    auto target_kw = this->get_parameter("target_keywords").as_string_array();
    auto obstacle_kw = this->get_parameter("obstacle_keywords").as_string_array();

    double fov_h = this->get_parameter("fov_horizontal_deg").as_double() * M_PI / 180.0;
    double fov_v = this->get_parameter("fov_vertical_deg").as_double() * M_PI / 180.0;
    double tan_half_fov_h = std::tan(fov_h / 2.0);
    double tan_half_fov_v = std::tan(fov_v / 2.0);
    double min_z = this->get_parameter("min_depth").as_double();
    double max_z = this->get_parameter("max_depth").as_double();

    double dim_x = this->get_parameter("dummy_size_x").as_double();
    double dim_y = this->get_parameter("dummy_size_y").as_double();
    double dim_z = this->get_parameter("dummy_size_z").as_double();

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

      geometry_msgs::msg::Point low_left;
      geometry_msgs::msg::Point top_right;

      tf2::Vector3 gz_corners[8];

      // Use real absolute Bounding Boxes from Gazebo plugin if available
      if (model_bboxes_.find(model_name) != model_bboxes_.end()) {
        const auto & bmin = model_bboxes_[model_name].first;
        const auto & bmax = model_bboxes_[model_name].second;

        gz_corners[0] = tf2::Vector3(bmin.x, bmin.y, bmin.z);
        gz_corners[1] = tf2::Vector3(bmax.x, bmin.y, bmin.z);
        gz_corners[2] = tf2::Vector3(bmin.x, bmax.y, bmin.z);
        gz_corners[3] = tf2::Vector3(bmax.x, bmax.y, bmin.z);
        gz_corners[4] = tf2::Vector3(bmin.x, bmin.y, bmax.z);
        gz_corners[5] = tf2::Vector3(bmax.x, bmin.y, bmax.z);
        gz_corners[6] = tf2::Vector3(bmin.x, bmax.y, bmax.z);
        gz_corners[7] = tf2::Vector3(bmax.x, bmax.y, bmax.z);
      } else {
        // Fallback to dummy sizes if plugin data is missing
        double gz_min_x = msg->pose[i].position.x - (dim_x / 2.0);
        double gz_min_y = msg->pose[i].position.y - (dim_y / 2.0);
        double gz_min_z = msg->pose[i].position.z;
        double gz_max_x = msg->pose[i].position.x + (dim_x / 2.0);
        double gz_max_y = msg->pose[i].position.y + (dim_y / 2.0);
        double gz_max_z = msg->pose[i].position.z + dim_z;

        gz_corners[0] = tf2::Vector3(gz_min_x, gz_min_y, gz_min_z);
        gz_corners[1] = tf2::Vector3(gz_max_x, gz_min_y, gz_min_z);
        gz_corners[2] = tf2::Vector3(gz_min_x, gz_max_y, gz_min_z);
        gz_corners[3] = tf2::Vector3(gz_max_x, gz_max_y, gz_min_z);
        gz_corners[4] = tf2::Vector3(gz_min_x, gz_min_y, gz_max_z);
        gz_corners[5] = tf2::Vector3(gz_max_x, gz_min_y, gz_max_z);
        gz_corners[6] = tf2::Vector3(gz_min_x, gz_max_y, gz_max_z);
        gz_corners[7] = tf2::Vector3(gz_max_x, gz_max_y, gz_max_z);
      }

      // Transform the 8 corners of the Gazebo AABB into the robot-centered MoveIt frame
      double min_x = std::numeric_limits<double>::infinity();
      double min_y = std::numeric_limits<double>::infinity();
      double min_z_val = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      double max_y = -std::numeric_limits<double>::infinity();
      double max_z_val = -std::numeric_limits<double>::infinity();

      for (int c = 0; c < 8; ++c) {
        tf2::Vector3 pt_moveit = robot_tf_inv * gz_corners[c];
        min_x = std::min(min_x, pt_moveit.x());
        min_y = std::min(min_y, pt_moveit.y());
        min_z_val = std::min(min_z_val, pt_moveit.z());
        max_x = std::max(max_x, pt_moveit.x());
        max_y = std::max(max_y, pt_moveit.y());
        max_z_val = std::max(max_z_val, pt_moveit.z());
      }

      low_left.x = min_x;
      low_left.y = min_y;
      low_left.z = min_z_val;

      top_right.x = max_x;
      top_right.y = max_y;
      top_right.z = max_z_val;

      // Volumetric Frustum Culling: test the 8 corners of the 3D AABB against all 6 frustum planes
      geometry_msgs::msg::Point corners_world[8];
      corners_world[0].x = low_left.x;  corners_world[0].y = low_left.y;  corners_world[0].z = low_left.z;
      corners_world[1].x = top_right.x; corners_world[1].y = low_left.y;  corners_world[1].z = low_left.z;
      corners_world[2].x = low_left.x;  corners_world[2].y = top_right.y; corners_world[2].z = low_left.z;
      corners_world[3].x = top_right.x; corners_world[3].y = top_right.y; corners_world[3].z = low_left.z;
      corners_world[4].x = low_left.x;  corners_world[4].y = low_left.y;  corners_world[4].z = top_right.z;
      corners_world[5].x = top_right.x; corners_world[5].y = low_left.y;  corners_world[5].z = top_right.z;
      corners_world[6].x = low_left.x;  corners_world[6].y = top_right.y; corners_world[6].z = top_right.z;
      corners_world[7].x = top_right.x; corners_world[7].y = top_right.y; corners_world[7].z = top_right.z;

      bool all_out_near = true;
      bool all_out_far = true;
      bool all_out_left = true;
      bool all_out_right = true;
      bool all_out_top = true;
      bool all_out_bottom = true;
      bool transform_ok = true;

      for (size_t c = 0; c < 8; ++c) {
        geometry_msgs::msg::PointStamped pt_global;
        pt_global.header.frame_id = global_frame;
        pt_global.point = corners_world[c];

        geometry_msgs::msg::PointStamped pt_camera;
        try {
          tf2::doTransform(pt_global, pt_camera, t);
        } catch (const tf2::TransformException & ex) {
          transform_ok = false;
          break;
        }

        double cx = pt_camera.point.x;
        double cy = pt_camera.point.y;
        double cz = pt_camera.point.z;

        double max_x_at_z = cz * tan_half_fov_h;
        double max_y_at_z = cz * tan_half_fov_v;

        if (cz >= min_z) all_out_near = false;
        if (cz <= max_z) all_out_far = false;
        if (cx >= -max_x_at_z) all_out_left = false;
        if (cx <= max_x_at_z) all_out_right = false;
        if (cy >= -max_y_at_z) all_out_top = false;
        if (cy <= max_y_at_z) all_out_bottom = false;
      }

      if (!transform_ok || all_out_near || all_out_far ||
          all_out_left || all_out_right || all_out_top || all_out_bottom) {
        continue;
      }

      // Object passed culling: store its AABB in `global_frame` (robot model frame, NOT Gazebo world)
      custom_messages::msg::Object obj;
      obj.target = is_target;
      
      // Bottom left corner (min)
      obj.shape.low_left = low_left;

      // Top right corner (max)
      obj.shape.top_right = top_right;

      out_map.objects.push_back(obj);
    }

    if (out_map.objects.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No objects currently inside camera frustum; publishing empty objects list.");
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
