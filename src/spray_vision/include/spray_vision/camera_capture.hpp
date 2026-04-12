#ifndef SPRAY_VISION__CAMERA_CAPTURE_HPP_
#define SPRAY_VISION__CAMERA_CAPTURE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace spray_vision
{

struct Frame
{
  int width;
  int height;
  std::vector<std::uint8_t> data;
};

class CameraCapture
{
public:
  CameraCapture(int frame_width, int frame_height, const std::string & camera_device);
  ~CameraCapture();
  bool capture_frame(Frame & frame);
  const std::string & last_error() const;

private:
  struct Impl;

  int frame_width_;
  int frame_height_;
  std::string camera_device_;
  std::string last_error_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace spray_vision

#endif  // SPRAY_VISION__CAMERA_CAPTURE_HPP_
