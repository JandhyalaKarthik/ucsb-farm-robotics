#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "spray_vision/camera_capture.hpp"
#include "spray_vision/detection.hpp"
#include "spray_vision/hailo_inference.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using spray_vision::CameraCapture;
using spray_vision::Detection;
using spray_vision::Frame;
using spray_vision::HailoInference;

namespace
{

struct Track
{
  int id;
  float center_x;
  float center_y;
  float previous_center_y;
  rclcpp::Time last_seen;
  bool has_triggered;
};

float center_x(const Detection & detection)
{
  return (detection.xmin + detection.xmax) * 0.5F;
}

float center_y(const Detection & detection)
{
  return (detection.ymin + detection.ymax) * 0.5F;
}

float squared_distance(float ax, float ay, float bx, float by)
{
  const float dx = ax - bx;
  const float dy = ay - by;
  return dx * dx + dy * dy;
}

}  // namespace

class SprayDetectorNode : public rclcpp::Node
{
public:
  SprayDetectorNode()
  : Node("spray_detector"),
    trigger_y_(declare_parameter<double>("trigger_y", 240.0)),
    confidence_threshold_(declare_parameter<double>("confidence_threshold", 0.5)),
    target_class_id_(declare_parameter<int>("target_class_id", 0)),
    cooldown_ms_(declare_parameter<int>("cooldown_ms", 1000)),
    // Delay between crossing the trigger line and firing the nozzle command.
    spray_delay_ms_(declare_parameter<int>("spray_delay_ms", 0)),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    use_fake_detections_(declare_parameter<bool>("use_fake_detections", true)),
    enable_debug_logging_(declare_parameter<bool>("enable_debug_logging", false)),
    hef_path_(declare_parameter<std::string>("hef_path", "")),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0"))
  {
    spray_cmd_pub_ = create_publisher<std_msgs::msg::Bool>("/spray_cmd", 10);
    if (!use_fake_detections_) {
      camera_capture_ = std::make_unique<CameraCapture>(frame_width_, frame_height_, camera_device_);
      hailo_inference_ = std::make_unique<HailoInference>(hef_path_, enable_debug_logging_);
    }

    timer_ = create_wall_timer(100ms, std::bind(&SprayDetectorNode::process_frame, this));
    last_trigger_time_ = get_clock()->now() - rclcpp::Duration::from_seconds(60.0);

    RCLCPP_INFO(get_logger(), "Spray detector initialized");
  }

private:
  void process_frame()
  {
    const auto now = get_clock()->now();
    // Fire any nozzle commands whose requested delay has elapsed before processing the next frame.
    publish_due_spray_commands(now);

    const auto detections = get_detections();
    const auto filtered = filter_detections(detections);

    update_tracks(filtered, now);
    expire_old_tracks(now);
  }

  std::vector<Detection> get_detections()
  {
    if (!use_fake_detections_) {
      Frame frame;
      std::vector<Detection> detections;

      if (!camera_capture_ || !camera_capture_->capture_frame(frame)) {
        if (enable_debug_logging_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Camera capture failed: %s",
            camera_capture_ ? camera_capture_->last_error().c_str() : "camera wrapper not initialized");
        }
        return {};
      }

      if (!hailo_inference_ || !hailo_inference_->infer(frame, detections)) {
        if (enable_debug_logging_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Hailo inference failed: %s",
            hailo_inference_ ? hailo_inference_->last_error().c_str() :
            "Hailo wrapper not initialized");
        }
        return {};
      }

      return detections;
    }

    fake_center_y_ += 18.0F;
    if (fake_center_y_ > static_cast<float>(frame_height_ + 50)) {
      fake_center_y_ = 40.0F;
    }

    const float box_width = 80.0F;
    const float box_height = 60.0F;
    const float fake_center_x = static_cast<float>(frame_width_) * 0.5F;

    return {Detection{
      fake_center_x - box_width * 0.5F,
      fake_center_y_ - box_height * 0.5F,
      fake_center_x + box_width * 0.5F,
      fake_center_y_ + box_height * 0.5F,
      0.90F,
      target_class_id_}};
  }

  std::vector<Detection> filter_detections(const std::vector<Detection> & detections) const
  {
    std::vector<Detection> filtered;
    for (const auto & detection : detections) {
      if (detection.class_id == target_class_id_ &&
        detection.confidence >= static_cast<float>(confidence_threshold_))
      {
        filtered.push_back(detection);
      }
    }
    return filtered;
  }

  void update_tracks(const std::vector<Detection> & detections, const rclcpp::Time & now)
  {
    std::vector<bool> matched_tracks(tracks_.size(), false);
    constexpr float max_match_distance = 80.0F;
    constexpr float max_match_distance_sq = max_match_distance * max_match_distance;

    for (const auto & detection : detections) {
      const float detection_center_x = center_x(detection);
      const float detection_center_y = center_y(detection);
      auto best_track = find_best_track(detection_center_x, detection_center_y, matched_tracks);

      if (best_track && best_track->second <= max_match_distance_sq) {
        update_track(best_track->first, detection_center_x, detection_center_y, now);
        matched_tracks[best_track->first] = true;
      } else {
        tracks_.push_back(Track{
          next_track_id_++,
          detection_center_x,
          detection_center_y,
          detection_center_y,
          now,
          false});
        matched_tracks.push_back(true);
      }
    }
  }

  std::optional<std::pair<std::size_t, float>> find_best_track(
    float detection_center_x,
    float detection_center_y,
    const std::vector<bool> & matched_tracks) const
  {
    std::optional<std::pair<std::size_t, float>> best_track;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (matched_tracks[i] || tracks_[i].has_triggered) {
        continue;
      }

      const float distance = squared_distance(
        detection_center_x, detection_center_y, tracks_[i].center_x, tracks_[i].center_y);
      if (!best_track || distance < best_track->second) {
        best_track = std::make_pair(i, distance);
      }
    }
    return best_track;
  }

  void update_track(
    std::size_t track_index,
    float detection_center_x,
    float detection_center_y,
    const rclcpp::Time & now)
  {
    auto & track = tracks_[track_index];
    track.previous_center_y = track.center_y;
    track.center_x = detection_center_x;
    track.center_y = detection_center_y;
    track.last_seen = now;

    // A track can trigger once, when its center moves past trigger_y and the cooldown allows it.
    if (should_trigger(track, now)) {
      schedule_spray_command(now);
      track.has_triggered = true;
      last_trigger_time_ = now;
    }
  }

  bool should_trigger(const Track & track, const rclcpp::Time & now) const
  {
    // Image Y increases downward, so crossing happens when the plant center moves below trigger_y.
    const bool crossed_trigger_line =
      track.previous_center_y < static_cast<float>(trigger_y_) &&
      track.center_y >= static_cast<float>(trigger_y_);
    const bool cooldown_expired =
      now - last_trigger_time_ >= rclcpp::Duration(std::chrono::milliseconds(cooldown_ms_));

    return crossed_trigger_line && cooldown_expired && !track.has_triggered;
  }

  void schedule_spray_command(const rclcpp::Time & now)
  {
    // Preserve old behavior when no delay is configured.
    if (spray_delay_ms_ <= 0) {
      publish_spray_command();
      return;
    }

    // Store the exact ROS time when the spray command should be published.
    pending_spray_times_.push_back(
      now + rclcpp::Duration(std::chrono::milliseconds(spray_delay_ms_)));
  }

  void publish_due_spray_commands(const rclcpp::Time & now)
  {
    // Publish all expired scheduled sprays and remove them from the pending list.
    pending_spray_times_.erase(
      std::remove_if(
        pending_spray_times_.begin(), pending_spray_times_.end(),
        [&](const rclcpp::Time & spray_time) {
          if ((now - spray_time).nanoseconds() < 0) {
            return false;
          }
          publish_spray_command();
          return true;
        }),
      pending_spray_times_.end());
  }

  void publish_spray_command()
  {
    // The downstream nozzle controller listens for true messages on /spray_cmd.
    std_msgs::msg::Bool msg;
    msg.data = true;
    spray_cmd_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published spray command");
  }

  void expire_old_tracks(const rclcpp::Time & now)
  {
    constexpr double max_track_age_seconds = 1.0;
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & track) {
          return (now - track.last_seen).seconds() > max_track_age_seconds;
        }),
      tracks_.end());
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<CameraCapture> camera_capture_;
  std::unique_ptr<HailoInference> hailo_inference_;
  std::vector<Track> tracks_;
  std::vector<rclcpp::Time> pending_spray_times_;
  rclcpp::Time last_trigger_time_;

  double trigger_y_;
  double confidence_threshold_;
  int target_class_id_;
  int cooldown_ms_;
  int spray_delay_ms_;
  int frame_width_;
  int frame_height_;
  bool use_fake_detections_;
  bool enable_debug_logging_;
  std::string hef_path_;
  std::string camera_device_;

  int next_track_id_{1};
  float fake_center_y_{40.0F};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SprayDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
