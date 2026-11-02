#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <custom_messages/msg/map.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <vector>
#include <algorithm>

namespace cr5_gazebo
{

class BboxImageOverlayNode : public rclcpp::Node
{
public:
  explicit BboxImageOverlayNode(const rclcpp::NodeOptions & options)
  : Node("bbox_image_overlay", options)
  {
    // Setup TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber to the Map (/Boing)
    map_sub_ = this->create_subscription<custom_messages::msg::Map>(
      "/Boing", 10,
      std::bind(&BboxImageOverlayNode::mapCallback, this, std::placeholders::_1)
    );

    // Publisher for annotated image
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/zed/camera/image_annotated", 10);

    // Setup message filters for image and camera info
    image_sub_.subscribe(this, "/zed/camera/image_raw");
    cam_info_sub_.subscribe(this, "/zed/camera/camera_info");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(
      image_sub_, cam_info_sub_, 10);
    sync_->registerCallback(std::bind(&BboxImageOverlayNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Bbox Image Overlay Node started.");
  }

private:
  void mapCallback(const custom_messages::msg::Map::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = msg;
  }

  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg)
  {
    custom_messages::msg::Map::SharedPtr current_map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!latest_map_) return; // No map data yet
      current_map = latest_map_;
    }

    // Convert ROS Image to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Extract Camera Intrinsics
    // K is a 3x3 array [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    double fx = cam_info_msg->k[0];
    double cx = cam_info_msg->k[2];
    double fy = cam_info_msg->k[4];
    double cy = cam_info_msg->k[5];

    std::string camera_frame = cam_info_msg->header.frame_id;
    if (camera_frame.empty() || camera_frame == "zed_camera") {
        camera_frame = "zed_optical_link";
    }

    // Get Transform from World to Camera
    geometry_msgs::msg::TransformStamped t;
    try {
      // Assuming map coordinates are in "world"
      t = tf_buffer_->lookupTransform(camera_frame, "world", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF Error: %s", ex.what());
      return;
    }

    // Draw each object
    for (const auto& obj : current_map->objects) {
      // 1. Reconstruct 8 corners of the 3D bounding box in World frame
      double min_x = obj.shape.low_left.x;
      double min_y = obj.shape.low_left.y;
      double min_z = obj.shape.low_left.z;
      
      double max_x = obj.shape.top_right.x;
      double max_y = obj.shape.top_right.y;
      double max_z = obj.shape.top_right.z;

      std::vector<geometry_msgs::msg::Point> corners_world(8);
      corners_world[0].x = min_x; corners_world[0].y = min_y; corners_world[0].z = min_z;
      corners_world[1].x = max_x; corners_world[1].y = min_y; corners_world[1].z = min_z;
      corners_world[2].x = min_x; corners_world[2].y = max_y; corners_world[2].z = min_z;
      corners_world[3].x = max_x; corners_world[3].y = max_y; corners_world[3].z = min_z;
      corners_world[4].x = min_x; corners_world[4].y = min_y; corners_world[4].z = max_z;
      corners_world[5].x = max_x; corners_world[5].y = min_y; corners_world[5].z = max_z;
      corners_world[6].x = min_x; corners_world[6].y = max_y; corners_world[6].z = max_z;
      corners_world[7].x = max_x; corners_world[7].y = max_y; corners_world[7].z = max_z;

      // 2. Transform 8 corners to Camera Frame
      std::vector<geometry_msgs::msg::Point> corners_cam(8);
      for (size_t i = 0; i < 8; ++i) {
        geometry_msgs::msg::PointStamped pt_global;
        pt_global.header.frame_id = "world";
        pt_global.point = corners_world[i];

        geometry_msgs::msg::PointStamped pt_cam;
        tf2::doTransform(pt_global, pt_cam, t);
        corners_cam[i] = pt_cam.point;
      }

      // Define 12 edges of the cuboid connecting the 8 corners
      const int edges[12][2] = {
        {0, 1}, {1, 3}, {3, 2}, {2, 0}, // Bottom face
        {4, 5}, {5, 7}, {7, 6}, {6, 4}, // Top face
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Vertical pillars
      };

      const double z_near = 0.10; // ZED camera near clipping distance in meters
      std::vector<cv::Point> pts_2d;

      for (int e = 0; e < 12; ++e) {
        const auto& p1 = corners_cam[edges[e][0]];
        const auto& p2 = corners_cam[edges[e][1]];

        bool p1_in = (p1.z >= z_near);
        bool p2_in = (p2.z >= z_near);

        if (!p1_in && !p2_in) {
          // Entire edge is behind or too close to camera plane
          continue;
        }

        if (p1_in && p2_in) {
          // Both vertices are in front of near plane
          pts_2d.push_back(cv::Point(
            static_cast<int>((p1.x / p1.z) * fx + cx),
            static_cast<int>((p1.y / p1.z) * fy + cy)
          ));
          pts_2d.push_back(cv::Point(
            static_cast<int>((p2.x / p2.z) * fx + cx),
            static_cast<int>((p2.y / p2.z) * fy + cy)
          ));
        } else {
          // Edge intersects near plane: interpolate point at z = z_near
          double t_param = (z_near - p1.z) / (p2.z - p1.z);
          double clip_x = p1.x + t_param * (p2.x - p1.x);
          double clip_y = p1.y + t_param * (p2.y - p1.y);

          // Project intersection point
          pts_2d.push_back(cv::Point(
            static_cast<int>((clip_x / z_near) * fx + cx),
            static_cast<int>((clip_y / z_near) * fy + cy)
          ));

          // Project the endpoint that is in front of the near plane
          const auto& p_valid = p1_in ? p1 : p2;
          pts_2d.push_back(cv::Point(
            static_cast<int>((p_valid.x / p_valid.z) * fx + cx),
            static_cast<int>((p_valid.y / p_valid.z) * fy + cy)
          ));
        }
      }

      if (pts_2d.empty()) continue;

      // 3. Find 2D Bounding Box
      int u_min = image_msg->width;
      int v_min = image_msg->height;
      int u_max = 0;
      int v_max = 0;

      for (const auto& p : pts_2d) {
        if (p.x < u_min) u_min = p.x;
        if (p.x > u_max) u_max = p.x;
        if (p.y < v_min) v_min = p.y;
        if (p.y > v_max) v_max = p.y;
      }

      // Clamp to image boundaries
      u_min = std::max(0, u_min);
      v_min = std::max(0, v_min);
      u_max = std::min(static_cast<int>(image_msg->width - 1), u_max);
      v_max = std::min(static_cast<int>(image_msg->height - 1), v_max);

      // Draw
      if (u_max > u_min && v_max > v_min) {
        cv::Scalar color = obj.target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255); // Green for target, Red for obstacle
        cv::rectangle(cv_ptr->image, cv::Point(u_min, v_min), cv::Point(u_max, v_max), color, 2);
        
        // Add label
        std::string label = obj.target ? "Target" : "Obstacle";
        cv::putText(cv_ptr->image, label, cv::Point(u_min, v_min - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
      }
    }

    // Publish Annotated Image
    image_pub_->publish(*cv_ptr->toImageMsg());
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Subscription<custom_messages::msg::Map>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cam_info_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> sync_;

  custom_messages::msg::Map::SharedPtr latest_map_;
  std::mutex map_mutex_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(cr5_gazebo::BboxImageOverlayNode)
