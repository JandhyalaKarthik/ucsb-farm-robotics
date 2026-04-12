#include "spray_vision/hailo_inference.hpp"

namespace spray_vision
{

HailoInference::HailoInference(
  const std::string & hef_path,
  bool enable_debug_logging)
: hef_path_(hef_path),
  enable_debug_logging_(enable_debug_logging)
{
}

bool HailoInference::infer(const Frame & frame, std::vector<Detection> & detections)
{
  (void)frame;
  detections.clear();

  // TODO: Bind HailoRT here once the deployed .hef tensor names/layout are known.
  // Expected output should be converted into Detection{xmin, ymin, xmax, ymax, confidence, class_id}.
  last_error_ = "HailoRT inference wrapper is present, but model tensor parsing is not configured";
  if (enable_debug_logging_ && hef_path_.empty()) {
    last_error_ = "HailoRT inference wrapper requires a hef_path parameter";
  }
  return false;
}

const std::string & HailoInference::last_error() const
{
  return last_error_;
}

}  // namespace spray_vision
