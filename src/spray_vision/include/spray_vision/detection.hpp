#ifndef SPRAY_VISION__DETECTION_HPP_
#define SPRAY_VISION__DETECTION_HPP_

namespace spray_vision
{

struct Detection
{
  float xmin;
  float ymin;
  float xmax;
  float ymax;
  float confidence;
  int class_id;
};

}  // namespace spray_vision

#endif  // SPRAY_VISION__DETECTION_HPP_
