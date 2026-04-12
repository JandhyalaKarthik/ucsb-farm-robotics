#ifndef SPRAY_VISION__HAILO_INFERENCE_HPP_
#define SPRAY_VISION__HAILO_INFERENCE_HPP_

#include <string>
#include <vector>

#include "spray_vision/camera_capture.hpp"
#include "spray_vision/detection.hpp"

namespace spray_vision
{

class HailoInference
{
public:
  HailoInference(const std::string & hef_path, bool enable_debug_logging);
  bool infer(const Frame & frame, std::vector<Detection> & detections);
  const std::string & last_error() const;

private:
  std::string hef_path_;
  bool enable_debug_logging_;
  std::string last_error_;
};

}  // namespace spray_vision

#endif  // SPRAY_VISION__HAILO_INFERENCE_HPP_
