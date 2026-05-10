#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace
{
double distance3d(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isFinitePoint(const geometry_msgs::msg::Point& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

tf2::Quaternion rotationBetweenVectors(const tf2::Vector3& from_vec, const tf2::Vector3& to_vec)
{
  tf2::Vector3 from = from_vec.normalized();
  tf2::Vector3 to = to_vec.normalized();
  const double dot = std::clamp(from.dot(to), -1.0, 1.0);

  if (dot > 0.999999) {
    return tf2::Quaternion::getIdentity();
  }

  if (dot < -0.999999) {
    tf2::Vector3 orthogonal = tf2::Vector3(1.0, 0.0, 0.0).cross(from);
    if (orthogonal.length2() < 1e-12) {
      orthogonal = tf2::Vector3(0.0, 1.0, 0.0).cross(from);
    }
    orthogonal.normalize();
    tf2::Quaternion q;
    q.setRotation(orthogonal, M_PI);
    return q;
  }

  tf2::Vector3 axis = from.cross(to);
  axis.normalize();
  tf2::Quaternion q;
  q.setRotation(axis, std::acos(dot));
  return q;
}

geometry_msgs::msg::Quaternion faceTargetQuaternion(const geometry_msgs::msg::Quaternion& current_orientation,
                                                    const tf2::Vector3& desired_forward)
{
  tf2::Quaternion current_q;
  tf2::fromMsg(current_orientation, current_q);
  tf2::Matrix3x3 current_basis(current_q);
  const tf2::Vector3 current_forward = current_basis * tf2::Vector3(0.0, 0.0, 1.0);
  const tf2::Quaternion align_q = rotationBetweenVectors(current_forward, desired_forward);
  tf2::Quaternion desired_q = align_q * current_q;
  desired_q.normalize();
  return tf2::toMsg(desired_q);
}
}  // namespace

class TargetApproachNode
{
public:
  explicit TargetApproachNode(const rclcpp::Node::SharedPtr& node)
    : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
    planning_frame_ = target_frame_.empty() ? move_group_->getPlanningFrame() : target_frame_;
    if (planning_frame_.empty()) {
      planning_frame_ = move_group_->getPlanningFrame();
    }

    if (!ee_link_.empty()) {
      move_group_->setEndEffectorLink(ee_link_);
    } else {
      ee_link_ = move_group_->getEndEffectorLink();
    }

    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    move_group_->setPlanningTime(planning_timeout_);
    move_group_->setNumPlanningAttempts(planning_attempts_);

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetApproachNode::cameraInfoCallback, this, std::placeholders::_1), subscription_options);
    color_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetApproachNode::colorCallback, this, std::placeholders::_1), subscription_options);
    depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetApproachNode::depthCallback, this, std::placeholders::_1), subscription_options);
    detections_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic_, rclcpp::QoS(10),
      std::bind(&TargetApproachNode::detectionsCallback, this, std::placeholders::_1), subscription_options);

    RCLCPP_INFO(node_->get_logger(), "Target approach node ready");
    RCLCPP_INFO(node_->get_logger(), "MoveIt group='%s', planning_frame='%s', ee_link='%s'",
                planning_group_.c_str(), planning_frame_.c_str(), ee_link_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Camera frame='%s', target_label='%s', dry_run=%s",
                camera_frame_.c_str(), target_label_.c_str(), dry_run_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Safe motion scaling: velocity=%.2f, acceleration=%.2f",
                max_velocity_scaling_, max_acceleration_scaling_);
    RCLCPP_INFO(node_->get_logger(), "Planning target mode: position_only_target=%s",
                position_only_target_ ? "true" : "false");
  }

private:
  struct DetectionCandidate
  {
    double score{ 0.0 };
    double u{ 0.0 };
    double v{ 0.0 };
    double size_x{ 0.0 };
    double size_y{ 0.0 };
    double tracking_cost{ 0.0 };
  };

  void declareParameters()
  {
    node_->declare_parameter<std::string>("planning_group", "gluon");
    node_->declare_parameter<std::string>("target_label", "");
    node_->declare_parameter<double>("score_threshold", 0.5);
    node_->declare_parameter<std::string>("color_topic", "/camera/camera/color/image_raw");
    node_->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    node_->declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    node_->declare_parameter<std::string>("detections_topic", "/detector/detections");
    node_->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    node_->declare_parameter<std::string>("target_frame", "");
    node_->declare_parameter<std::string>("planning_frame", "");
    node_->declare_parameter<std::string>("ee_link", "6_Link");
    node_->declare_parameter<double>("approach_distance", 0.18);
    node_->declare_parameter<double>("stop_distance", 0.16);
    node_->declare_parameter<double>("z_offset", 0.0);
    node_->declare_parameter<double>("lateral_offset", 0.0);
    node_->declare_parameter<int>("depth_window_size", 7);
    node_->declare_parameter<double>("min_depth", 0.05);
    node_->declare_parameter<double>("max_depth", 2.0);
    node_->declare_parameter<double>("tracking_score_weight", 2.0);
    node_->declare_parameter<double>("tracking_distance_weight", 1.0);
    node_->declare_parameter<double>("depth_sample_x_ratio", 0.5);
    node_->declare_parameter<double>("depth_sample_y_ratio", 0.62);
    node_->declare_parameter<double>("depth_sample_y_bias_max_px", 24.0);
    node_->declare_parameter<int>("track_memory_frames", 15);
    node_->declare_parameter<int>("track_max_pixel_jump", 180);
    node_->declare_parameter<int>("target_filter_window_size", 5);
    node_->declare_parameter<double>("target_filter_alpha", 0.35);
    node_->declare_parameter<double>("target_filter_max_jump", 0.12);
    node_->declare_parameter<int>("stable_required_frames", 5);
    node_->declare_parameter<double>("stable_position_tolerance", 0.03);
    node_->declare_parameter<double>("min_x", 0.05);
    node_->declare_parameter<double>("max_x", 0.65);
    node_->declare_parameter<double>("min_y", -0.35);
    node_->declare_parameter<double>("max_y", 0.35);
    node_->declare_parameter<double>("min_z", 0.02);
    node_->declare_parameter<double>("max_z", 0.55);
    node_->declare_parameter<bool>("dry_run", true);
    node_->declare_parameter<bool>("position_only_target", false);
    node_->declare_parameter<bool>("align_camera_to_target", true);
    node_->declare_parameter<double>("max_velocity_scaling", 0.25);
    node_->declare_parameter<double>("max_acceleration_scaling", 0.25);
    node_->declare_parameter<double>("planning_timeout", 8.0);
    node_->declare_parameter<int>("planning_attempts", 5);
  }

  void loadParameters()
  {
    planning_group_ = node_->get_parameter("planning_group").as_string();
    target_label_ = node_->get_parameter("target_label").as_string();
    score_threshold_ = node_->get_parameter("score_threshold").as_double();
    color_topic_ = node_->get_parameter("color_topic").as_string();
    camera_info_topic_ = node_->get_parameter("camera_info_topic").as_string();
    depth_topic_ = node_->get_parameter("depth_topic").as_string();
    detections_topic_ = node_->get_parameter("detections_topic").as_string();
    camera_frame_ = node_->get_parameter("camera_frame").as_string();
    target_frame_ = node_->get_parameter("target_frame").as_string();
    const std::string planning_frame_param = node_->get_parameter("planning_frame").as_string();
    if (!planning_frame_param.empty()) {
      target_frame_ = planning_frame_param;
    }
    ee_link_ = node_->get_parameter("ee_link").as_string();
    approach_distance_ = node_->get_parameter("approach_distance").as_double();
    stop_distance_ = node_->get_parameter("stop_distance").as_double();
    z_offset_ = node_->get_parameter("z_offset").as_double();
    lateral_offset_ = node_->get_parameter("lateral_offset").as_double();
    depth_window_size_ = node_->get_parameter("depth_window_size").as_int();
    min_depth_ = node_->get_parameter("min_depth").as_double();
    max_depth_ = node_->get_parameter("max_depth").as_double();
    tracking_score_weight_ = node_->get_parameter("tracking_score_weight").as_double();
    tracking_distance_weight_ = node_->get_parameter("tracking_distance_weight").as_double();
    depth_sample_x_ratio_ = node_->get_parameter("depth_sample_x_ratio").as_double();
    depth_sample_y_ratio_ = node_->get_parameter("depth_sample_y_ratio").as_double();
    depth_sample_y_bias_max_px_ = node_->get_parameter("depth_sample_y_bias_max_px").as_double();
    track_memory_frames_ = node_->get_parameter("track_memory_frames").as_int();
    track_max_pixel_jump_ = node_->get_parameter("track_max_pixel_jump").as_int();
    target_filter_window_size_ = node_->get_parameter("target_filter_window_size").as_int();
    target_filter_alpha_ = node_->get_parameter("target_filter_alpha").as_double();
    target_filter_max_jump_ = node_->get_parameter("target_filter_max_jump").as_double();
    stable_required_frames_ = node_->get_parameter("stable_required_frames").as_int();
    stable_position_tolerance_ = node_->get_parameter("stable_position_tolerance").as_double();
    min_x_ = node_->get_parameter("min_x").as_double();
    max_x_ = node_->get_parameter("max_x").as_double();
    min_y_ = node_->get_parameter("min_y").as_double();
    max_y_ = node_->get_parameter("max_y").as_double();
    min_z_ = node_->get_parameter("min_z").as_double();
    max_z_ = node_->get_parameter("max_z").as_double();
    dry_run_ = node_->get_parameter("dry_run").as_bool();
    position_only_target_ = node_->get_parameter("position_only_target").as_bool();
    align_camera_to_target_ = node_->get_parameter("align_camera_to_target").as_bool();
    max_velocity_scaling_ = node_->get_parameter("max_velocity_scaling").as_double();
    max_acceleration_scaling_ = node_->get_parameter("max_acceleration_scaling").as_double();
    planning_timeout_ = node_->get_parameter("planning_timeout").as_double();
    planning_attempts_ = node_->get_parameter("planning_attempts").as_int();

    if (align_camera_to_target_ && position_only_target_) {
      RCLCPP_WARN(node_->get_logger(),
                  "align_camera_to_target=true requires a full pose target; overriding position_only_target=false");
      position_only_target_ = false;
    }

    max_velocity_scaling_ = clampScaling(max_velocity_scaling_, 0.05, 0.30, "max_velocity_scaling");
    max_acceleration_scaling_ = clampScaling(max_acceleration_scaling_, 0.05, 0.30, "max_acceleration_scaling");

    if (depth_window_size_ < 1) {
      depth_window_size_ = 7;
    }
    if (depth_window_size_ % 2 == 0) {
      ++depth_window_size_;
    }
    stable_required_frames_ = std::max(1, stable_required_frames_);
    tracking_score_weight_ = std::clamp(tracking_score_weight_, 0.0, 10.0);
    tracking_distance_weight_ = std::clamp(tracking_distance_weight_, 0.0, 10.0);
    depth_sample_x_ratio_ = std::clamp(depth_sample_x_ratio_, 0.1, 0.9);
    depth_sample_y_ratio_ = std::clamp(depth_sample_y_ratio_, 0.1, 0.9);
    depth_sample_y_bias_max_px_ = std::max(0.0, depth_sample_y_bias_max_px_);
    track_memory_frames_ = std::max(0, track_memory_frames_);
    track_max_pixel_jump_ = std::max(10, track_max_pixel_jump_);
    target_filter_window_size_ = std::max(1, target_filter_window_size_);
    target_filter_alpha_ = std::clamp(target_filter_alpha_, 0.05, 1.0);
    target_filter_max_jump_ = std::max(0.02, target_filter_max_jump_);
  }

  double clampScaling(double value, double min_value, double max_value, const std::string& name) const
  {
    if (!std::isfinite(value)) {
      RCLCPP_WARN(node_->get_logger(), "%s is not finite; using safe default %.2f", name.c_str(), min_value);
      return min_value;
    }
    if (value < min_value) {
      RCLCPP_WARN(node_->get_logger(), "%s=%.3f is below safe minimum %.3f; clamping", name.c_str(), value,
                  min_value);
      return min_value;
    }
    if (value > max_value) {
      RCLCPP_WARN(node_->get_logger(), "%s=%.3f is above safe maximum %.3f; clamping", name.c_str(), value,
                  max_value);
      return max_value;
    }
    return value;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    camera_model_.fromCameraInfo(msg);
    latest_camera_info_ = msg;
  }

  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_color_stamp_ = msg->header.stamp;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = msg;
  }

  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (motion_in_progress_) {
      RCLCPP_DEBUG(node_->get_logger(), "Ignoring detections while motion is in progress");
      return;
    }

    const auto candidate = selectBestDetection(*msg);
    if (!candidate) {
      tracking_memory_misses_ = std::min(track_memory_frames_ + 1, tracking_memory_misses_ + 1);
      if (tracking_memory_misses_ > track_memory_frames_) {
        last_detection_pixel_.reset();
      }
      resetStability("no matching detection");
      return;
    }

    sensor_msgs::msg::Image::SharedPtr depth_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    image_geometry::PinholeCameraModel camera_model;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      depth_msg = latest_depth_;
      camera_info_msg = latest_camera_info_;
      camera_model = camera_model_;
    }

    if (!depth_msg || !camera_info_msg) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Waiting for depth image and camera_info before approaching target");
      return;
    }

    const auto depth_m = extractMedianDepth(depth_msg, *candidate);
    if (!depth_m) {
      RCLCPP_WARN(node_->get_logger(), "Skipping detection: no valid depth near pixel (%.1f, %.1f)",
                  candidate->u, candidate->v);
      tracking_memory_misses_ = std::min(track_memory_frames_ + 1, tracking_memory_misses_ + 1);
      if (tracking_memory_misses_ > track_memory_frames_) {
        last_detection_pixel_.reset();
      }
      return;
    }
    geometry_msgs::msg::Point tracked_pixel;
    tracked_pixel.x = candidate->u;
    tracked_pixel.y = candidate->v;
    tracked_pixel.z = 0.0;
    last_detection_pixel_ = tracked_pixel;
    tracking_memory_misses_ = 0;

    const auto camera_point = pixelToCameraPoint(camera_model, *depth_msg, candidate->u, candidate->v, *depth_m);
    if (!camera_point) {
      return;
    }

    geometry_msgs::msg::PointStamped target_in_planning;
    geometry_msgs::msg::PointStamped camera_origin_in_planning;
    if (!transformTarget(*camera_point, target_in_planning, camera_origin_in_planning)) {
      return;
    }

    if (!isWorkspacePointAllowed(target_in_planning.point, "target")) {
      return;
    }

    const geometry_msgs::msg::Point raw_target_point = target_in_planning.point;
    target_in_planning.point = smoothTargetPoint(raw_target_point);

    if (!updateStability(target_in_planning.point)) {
      RCLCPP_INFO(node_->get_logger(),
                  "Target candidate score=%.3f at pixel=(%.1f, %.1f), depth=%.3f m; raw_xyz=(%.3f, %.3f, %.3f) filtered_xyz=(%.3f, %.3f, %.3f); stable %d/%d",
                  candidate->score, candidate->u, candidate->v, *depth_m,
                  raw_target_point.x, raw_target_point.y, raw_target_point.z,
                  target_in_planning.point.x, target_in_planning.point.y, target_in_planning.point.z,
                  stable_count_, stable_required_frames_);
      return;
    }

    planAndMaybeExecute(target_in_planning, camera_origin_in_planning, candidate->score);
  }

  std::optional<DetectionCandidate> selectBestDetection(const vision_msgs::msg::Detection2DArray& msg) const
  {
    std::optional<geometry_msgs::msg::Point> reference_pixel;
    double image_center_u = 0.0;
    double image_center_v = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (latest_camera_info_ && latest_camera_info_->width > 0 && latest_camera_info_->height > 0) {
        image_center_u = static_cast<double>(latest_camera_info_->width) * 0.5;
        image_center_v = static_cast<double>(latest_camera_info_->height) * 0.5;
      }
      if (last_detection_pixel_ && tracking_memory_misses_ <= track_memory_frames_) {
        reference_pixel = last_detection_pixel_;
      }
    }

    if (!reference_pixel && image_center_u > 0.0 && image_center_v > 0.0) {
      geometry_msgs::msg::Point center_pixel;
      center_pixel.x = image_center_u;
      center_pixel.y = image_center_v;
      center_pixel.z = 0.0;
      reference_pixel = center_pixel;
    }

    std::optional<DetectionCandidate> best;
    double best_cost = std::numeric_limits<double>::infinity();
    double image_scale = std::max(image_center_u * 2.0, image_center_v * 2.0);
    if (image_scale < 1.0) {
      image_scale = 640.0;
    }

    for (const auto& detection : msg.detections) {
      for (const auto& result : detection.results) {
        const auto& hypothesis = result.hypothesis;
        if (!target_label_.empty() && hypothesis.class_id != target_label_) {
          continue;
        }
        if (hypothesis.score < score_threshold_) {
          continue;
        }

        const double u = detection.bbox.center.position.x;
        const double v = detection.bbox.center.position.y;
        const double size_x = detection.bbox.size_x;
        const double size_y = detection.bbox.size_y;

        double distance_cost = 0.0;
        if (reference_pixel) {
          const double du = u - reference_pixel->x;
          const double dv = v - reference_pixel->y;
          const double pixel_distance = std::sqrt(du * du + dv * dv);
          if (last_detection_pixel_ && tracking_memory_misses_ <= track_memory_frames_ &&
              pixel_distance > static_cast<double>(track_max_pixel_jump_)) {
            continue;
          }
          distance_cost = pixel_distance / image_scale;
        }

        const double score_cost = 1.0 - hypothesis.score;
        const double total_cost = tracking_distance_weight_ * distance_cost +
                                  tracking_score_weight_ * score_cost;

        if (!best || total_cost < best_cost) {
          best_cost = total_cost;
          best = DetectionCandidate{ hypothesis.score, u, v, size_x, size_y, total_cost };
        }
      }
    }

    return best;
  }

  std::optional<double> extractMedianDepth(const sensor_msgs::msg::Image::SharedPtr& depth_msg,
                                                const DetectionCandidate& candidate) const
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception& ex) {
      RCLCPP_WARN(node_->get_logger(), "cv_bridge failed for depth image: %s", ex.what());
      return std::nullopt;
    }

    const double sample_u = candidate.u + (depth_sample_x_ratio_ - 0.5) * candidate.size_x;
    const double y_bias = std::min(depth_sample_y_bias_max_px_, std::max(0.0, candidate.size_y * (depth_sample_y_ratio_ - 0.5)));
    const double sample_v = candidate.v + y_bias;

    const int center_x = static_cast<int>(std::lround(sample_u));
    const int center_y = static_cast<int>(std::lround(sample_v));
    if (center_x < 0 || center_x >= cv_ptr->image.cols || center_y < 0 || center_y >= cv_ptr->image.rows) {
      RCLCPP_WARN(node_->get_logger(), "Detection pixel outside depth image: (%d, %d), image=%dx%d",
                  center_x, center_y, cv_ptr->image.cols, cv_ptr->image.rows);
      return std::nullopt;
    }

    const int adaptive_radius = std::max(depth_window_size_ / 2,
                                         static_cast<int>(std::lround(std::min(candidate.size_x, candidate.size_y) * 0.08)));
    const int radius = std::min(20, adaptive_radius);
    std::vector<double> depths;
    depths.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));

    for (int y = std::max(0, center_y - radius); y <= std::min(cv_ptr->image.rows - 1, center_y + radius); ++y) {
      for (int x = std::max(0, center_x - radius); x <= std::min(cv_ptr->image.cols - 1, center_x + radius); ++x) {
        double depth_m = std::numeric_limits<double>::quiet_NaN();
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
          depth_m = static_cast<double>(cv_ptr->image.at<uint16_t>(y, x)) * 0.001;
        } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
          depth_m = static_cast<double>(cv_ptr->image.at<float>(y, x));
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                               "Unsupported depth encoding '%s'; expected 16UC1 or 32FC1",
                               depth_msg->encoding.c_str());
          return std::nullopt;
        }

        if (std::isfinite(depth_m) && depth_m >= min_depth_ && depth_m <= max_depth_) {
          depths.push_back(depth_m);
        }
      }
    }

    if (depths.empty()) {
      return std::nullopt;
    }

    const auto middle = depths.begin() + static_cast<std::ptrdiff_t>(depths.size() / 2);
    std::nth_element(depths.begin(), middle, depths.end());
    return *middle;
  }

  std::optional<geometry_msgs::msg::PointStamped> pixelToCameraPoint(
    const image_geometry::PinholeCameraModel& camera_model,
    const sensor_msgs::msg::Image& depth_msg,
    double u,
    double v,
    double depth_m) const
  {
    if (!std::isfinite(depth_m) || depth_m <= 0.0) {
      return std::nullopt;
    }

    const cv::Point2d pixel(u, v);
    const cv::Point3d ray = camera_model.projectPixelTo3dRay(pixel);

    geometry_msgs::msg::PointStamped point;
    point.header.stamp = depth_msg.header.stamp;
    point.header.frame_id = camera_frame_.empty() ? depth_msg.header.frame_id : camera_frame_;
    point.point.x = ray.x * depth_m;
    point.point.y = ray.y * depth_m;
    point.point.z = ray.z * depth_m;

    RCLCPP_DEBUG(node_->get_logger(), "Camera point: frame=%s xyz=(%.3f, %.3f, %.3f)",
                 point.header.frame_id.c_str(), point.point.x, point.point.y, point.point.z);
    return point;
  }

  bool transformTarget(const geometry_msgs::msg::PointStamped& camera_point,
                       geometry_msgs::msg::PointStamped& target_in_planning,
                       geometry_msgs::msg::PointStamped& camera_origin_in_planning)
  {
    try {
      target_in_planning = tf_buffer_.transform(camera_point, planning_frame_, tf2::durationFromSec(0.2));

      geometry_msgs::msg::PointStamped camera_origin;
      camera_origin.header = camera_point.header;
      camera_origin.point.x = 0.0;
      camera_origin.point.y = 0.0;
      camera_origin.point.z = 0.0;
      camera_origin_in_planning = tf_buffer_.transform(camera_origin, planning_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "TF unavailable from '%s' to '%s': %s",
                  camera_point.header.frame_id.c_str(), planning_frame_.c_str(), ex.what());
      return false;
    }

    return true;
  }

  geometry_msgs::msg::Point smoothTargetPoint(const geometry_msgs::msg::Point& raw_point)
  {
    if (!filtered_target_point_ || distance3d(*filtered_target_point_, raw_point) > target_filter_max_jump_) {
      target_point_window_.clear();
      filtered_target_point_ = raw_point;
    }

    target_point_window_.push_back(raw_point);
    while (static_cast<int>(target_point_window_.size()) > target_filter_window_size_) {
      target_point_window_.pop_front();
    }

    geometry_msgs::msg::Point window_average;
    for (const auto& point : target_point_window_) {
      window_average.x += point.x;
      window_average.y += point.y;
      window_average.z += point.z;
    }
    const double denom = static_cast<double>(target_point_window_.size());
    window_average.x /= denom;
    window_average.y /= denom;
    window_average.z /= denom;

    if (!filtered_target_point_) {
      filtered_target_point_ = window_average;
    } else {
      filtered_target_point_->x = target_filter_alpha_ * window_average.x + (1.0 - target_filter_alpha_) * filtered_target_point_->x;
      filtered_target_point_->y = target_filter_alpha_ * window_average.y + (1.0 - target_filter_alpha_) * filtered_target_point_->y;
      filtered_target_point_->z = target_filter_alpha_ * window_average.z + (1.0 - target_filter_alpha_) * filtered_target_point_->z;
    }

    return *filtered_target_point_;
  }

  void resetTargetPointFilter(bool clear_history)
  {
    if (clear_history) {
      target_point_window_.clear();
      filtered_target_point_.reset();
    }
  }

  bool updateStability(const geometry_msgs::msg::Point& target_point)
  {
    if (!last_stable_point_ || distance3d(*last_stable_point_, target_point) <= stable_position_tolerance_) {
      ++stable_count_;
    } else {
      stable_count_ = 1;
    }
    last_stable_point_ = target_point;
    return stable_count_ >= stable_required_frames_;
  }

  void resetStability(const std::string& reason)
  {
    if (stable_count_ > 0) {
      RCLCPP_INFO(node_->get_logger(), "Reset target stability: %s", reason.c_str());
    }
    if (reason == "no matching detection") {
      resetTargetPointFilter(true);
    }
    stable_count_ = 0;
    last_stable_point_.reset();
  }

  bool isWorkspacePointAllowed(const geometry_msgs::msg::Point& point, const std::string& label) const
  {
    if (!isFinitePoint(point)) {
      RCLCPP_WARN(node_->get_logger(), "Rejecting %s point: non-finite coordinates", label.c_str());
      return false;
    }
    if (point.x < min_x_ || point.x > max_x_ || point.y < min_y_ || point.y > max_y_ || point.z < min_z_ ||
        point.z > max_z_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Rejecting %s point outside workspace: xyz=(%.3f, %.3f, %.3f), limits x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f]",
                  label.c_str(), point.x, point.y, point.z, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
      return false;
    }
    return true;
  }

  std::optional<geometry_msgs::msg::PoseStamped> makeApproachPose(
    const geometry_msgs::msg::PointStamped& target_in_planning,
    const geometry_msgs::msg::PointStamped& camera_origin_in_planning)
  {
    const double current_distance = distance3d(camera_origin_in_planning.point, target_in_planning.point);
    const double distance_error = current_distance - approach_distance_;
    if (std::abs(distance_error) <= stop_distance_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Target already within standoff tolerance: camera_to_target=%.3f m, desired=%.3f m, tolerance=%.3f m; keeping current position",
                  current_distance, approach_distance_, stop_distance_);
      return std::nullopt;
    }

    const double vx = target_in_planning.point.x - camera_origin_in_planning.point.x;
    const double vy = target_in_planning.point.y - camera_origin_in_planning.point.y;
    const double vz = target_in_planning.point.z - camera_origin_in_planning.point.z;
    const double norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (norm < 1e-6) {
      RCLCPP_WARN(node_->get_logger(), "Camera-to-target ray is degenerate; skipping motion");
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped desired_camera_pose;
    desired_camera_pose.header.frame_id = planning_frame_;
    desired_camera_pose.header.stamp = node_->now();
    desired_camera_pose.pose.position.x = target_in_planning.point.x - (vx / norm) * approach_distance_;
    desired_camera_pose.pose.position.y = target_in_planning.point.y - (vy / norm) * approach_distance_ + lateral_offset_;
    desired_camera_pose.pose.position.z = target_in_planning.point.z - (vz / norm) * approach_distance_ + z_offset_;

    try {
      const auto planning_to_camera_msg =
        tf_buffer_.lookupTransform(planning_frame_, camera_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
      const auto planning_to_ee_msg =
        tf_buffer_.lookupTransform(planning_frame_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(0.2));

      tf2::Transform T_planning_camera;
      tf2::Transform T_planning_ee;
      tf2::fromMsg(planning_to_camera_msg.transform, T_planning_camera);
      tf2::fromMsg(planning_to_ee_msg.transform, T_planning_ee);

      const tf2::Transform T_ee_camera = T_planning_ee.inverse() * T_planning_camera;

      if (align_camera_to_target_) {
        desired_camera_pose.pose.orientation =
          faceTargetQuaternion(planning_to_camera_msg.transform.rotation, tf2::Vector3(vx, vy, vz));
      } else {
        desired_camera_pose.pose.orientation = planning_to_camera_msg.transform.rotation;
      }

      tf2::Transform T_planning_camera_target;
      T_planning_camera_target.setOrigin(tf2::Vector3(desired_camera_pose.pose.position.x,
                                                      desired_camera_pose.pose.position.y,
                                                      desired_camera_pose.pose.position.z));
      T_planning_camera_target.setRotation(tf2::Quaternion(desired_camera_pose.pose.orientation.x,
                                                           desired_camera_pose.pose.orientation.y,
                                                           desired_camera_pose.pose.orientation.z,
                                                           desired_camera_pose.pose.orientation.w));

      const tf2::Transform T_planning_ee_target = T_planning_camera_target * T_ee_camera.inverse();

      geometry_msgs::msg::PoseStamped ee_target_pose;
      ee_target_pose.header = desired_camera_pose.header;
      ee_target_pose.pose.position.x = T_planning_ee_target.getOrigin().x();
      ee_target_pose.pose.position.y = T_planning_ee_target.getOrigin().y();
      ee_target_pose.pose.position.z = T_planning_ee_target.getOrigin().z();
      const tf2::Quaternion ee_target_q = T_planning_ee_target.getRotation();
      ee_target_pose.pose.orientation.x = ee_target_q.x();
      ee_target_pose.pose.orientation.y = ee_target_q.y();
      ee_target_pose.pose.orientation.z = ee_target_q.z();
      ee_target_pose.pose.orientation.w = ee_target_q.w();

      if (!isWorkspacePointAllowed(desired_camera_pose.pose.position, "approach_camera")) {
        return std::nullopt;
      }
      if (!isWorkspacePointAllowed(ee_target_pose.pose.position, "approach_ee")) {
        return std::nullopt;
      }

      RCLCPP_INFO(node_->get_logger(),
                  "Converted desired camera pose to ee target: camera[%s]=(%.3f, %.3f, %.3f) -> ee[%s]=(%.3f, %.3f, %.3f)",
                  planning_frame_.c_str(), desired_camera_pose.pose.position.x, desired_camera_pose.pose.position.y,
                  desired_camera_pose.pose.position.z, planning_frame_.c_str(), ee_target_pose.pose.position.x,
                  ee_target_pose.pose.position.y, ee_target_pose.pose.position.z);

      return ee_target_pose;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(),
                  "Unable to compute ee target from camera target using TF (%s -> %s): %s",
                  ee_link_.c_str(), camera_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void planAndMaybeExecute(const geometry_msgs::msg::PointStamped& target_in_planning,
                           const geometry_msgs::msg::PointStamped& camera_origin_in_planning,
                           double score)
  {
    bool expected_motion_state = false;
    if (!motion_in_progress_.compare_exchange_strong(expected_motion_state, true)) {
      RCLCPP_DEBUG(node_->get_logger(), "Ignoring stable target because motion is already in progress");
      return;
    }
    struct MotionGuard
    {
      TargetApproachNode* self;
      ~MotionGuard()
      {
        self->motion_in_progress_ = false;
        self->resetStability("motion attempt finished");
      }
    } motion_guard{ this };

    const auto approach_pose = makeApproachPose(target_in_planning, camera_origin_in_planning);
    if (!approach_pose) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Stable target accepted: score=%.3f target[%s]=(%.3f, %.3f, %.3f), approach_pose[%s]=(%.3f, %.3f, %.3f)",
                score, planning_frame_.c_str(), target_in_planning.point.x, target_in_planning.point.y,
                target_in_planning.point.z, approach_pose->header.frame_id.c_str(), approach_pose->pose.position.x,
                approach_pose->pose.position.y, approach_pose->pose.position.z);
    RCLCPP_INFO(node_->get_logger(),
                "Approach orientation xyzw=(%.4f, %.4f, %.4f, %.4f), dry_run=%s, position_only_target=%s",
                approach_pose->pose.orientation.x, approach_pose->pose.orientation.y, approach_pose->pose.orientation.z,
                approach_pose->pose.orientation.w, dry_run_ ? "true" : "false",
                position_only_target_ ? "true" : "false");

    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();
    if (position_only_target_) {
      move_group_->setPositionTarget(approach_pose->pose.position.x, approach_pose->pose.position.y,
                                     approach_pose->pose.position.z, ee_link_);
    } else {
      move_group_->setPoseTarget(*approach_pose, ee_link_);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group_->plan(plan);
    const bool plan_success = static_cast<bool>(result);
    RCLCPP_INFO(node_->get_logger(), "MoveIt plan result: %s", plan_success ? "SUCCESS" : "FAILED");

    if (!plan_success) {
      move_group_->clearPoseTargets();
      return;
    }

    if (dry_run_) {
      RCLCPP_INFO(node_->get_logger(), "dry_run=true: planned successfully, not executing trajectory");
      move_group_->clearPoseTargets();
      return;
    }

    const auto execute_result = move_group_->execute(plan);
    RCLCPP_INFO(node_->get_logger(), "MoveIt execute result: %s",
                static_cast<bool>(execute_result) ? "SUCCESS" : "FAILED");
    move_group_->clearPoseTargets();
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;

  mutable std::mutex data_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;
  rclcpp::Time latest_color_stamp_;

  std::string planning_group_;
  std::string target_label_;
  double score_threshold_{ 0.5 };
  std::string color_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string detections_topic_;
  std::string camera_frame_;
  std::string target_frame_;
  std::string planning_frame_;
  std::string ee_link_;
  double approach_distance_{ 0.18 };
  double stop_distance_{ 0.16 };
  double z_offset_{ 0.0 };
  double lateral_offset_{ 0.0 };
  int depth_window_size_{ 7 };
  double min_depth_{ 0.05 };
  double max_depth_{ 2.0 };
  double tracking_score_weight_{ 2.0 };
  double tracking_distance_weight_{ 1.0 };
  double depth_sample_x_ratio_{ 0.5 };
  double depth_sample_y_ratio_{ 0.62 };
  double depth_sample_y_bias_max_px_{ 24.0 };
  int track_memory_frames_{ 15 };
  int track_max_pixel_jump_{ 180 };
  int target_filter_window_size_{ 5 };
  double target_filter_alpha_{ 0.35 };
  double target_filter_max_jump_{ 0.12 };
  int stable_required_frames_{ 5 };
  double stable_position_tolerance_{ 0.03 };
  double min_x_{ 0.05 };
  double max_x_{ 0.65 };
  double min_y_{ -0.35 };
  double max_y_{ 0.35 };
  double min_z_{ 0.02 };
  double max_z_{ 0.55 };
  bool dry_run_{ true };
  bool position_only_target_{ false };
  bool align_camera_to_target_{ true };
  double max_velocity_scaling_{ 0.25 };
  double max_acceleration_scaling_{ 0.25 };
  double planning_timeout_{ 8.0 };
  int planning_attempts_{ 5 };

  std::atomic_bool motion_in_progress_{ false };
  int stable_count_{ 0 };
  int tracking_memory_misses_{ 0 };
  std::optional<geometry_msgs::msg::Point> last_stable_point_;
  std::optional<geometry_msgs::msg::Point> last_detection_pixel_;
  std::optional<geometry_msgs::msg::Point> filtered_target_point_;
  std::deque<geometry_msgs::msg::Point> target_point_window_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("target_approach_node");
  auto target_approach = std::make_shared<TargetApproachNode>(node);
  (void)target_approach;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
