#include "spray_vision/camera_capture.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>

#ifdef SPRAY_VISION_USE_OPENCV
#include <opencv2/opencv.hpp>
#else
#include <csignal>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace spray_vision
{

struct CameraCapture::Impl
{
#ifdef SPRAY_VISION_USE_OPENCV
  cv::VideoCapture capture;
#else
  ~Impl()
  {
    close_picamera2();
  }

  bool start_picamera2(int width, int height, const std::string & camera_device, std::string & error)
  {
    // Keep one Picamera2 helper alive so capture_frame() does not pay process startup cost every frame.
    if (picamera2_pid > 0) {
      return true;
    }

    std::signal(SIGPIPE, SIG_IGN);

    // request_pipe sends one byte per requested frame; frame_pipe returns raw RGB888 image bytes.
    int request_pipe[2] = {-1, -1};
    int frame_pipe[2] = {-1, -1};
    if (pipe(request_pipe) != 0 || pipe(frame_pipe) != 0) {
      error = std::string("failed to create Picamera2 pipes: ") + std::strerror(errno);
      close_fd(request_pipe[0]);
      close_fd(request_pipe[1]);
      close_fd(frame_pipe[0]);
      close_fd(frame_pipe[1]);
      return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
      error = std::string("failed to fork Picamera2 capture helper: ") + std::strerror(errno);
      close_fd(request_pipe[0]);
      close_fd(request_pipe[1]);
      close_fd(frame_pipe[0]);
      close_fd(frame_pipe[1]);
      return false;
    }

    if (pid == 0) {
      // Child process: connect stdin/stdout to the pipes and run the Picamera2 capture loop.
      dup2(request_pipe[0], STDIN_FILENO);
      dup2(frame_pipe[1], STDOUT_FILENO);
      close_fd(request_pipe[0]);
      close_fd(request_pipe[1]);
      close_fd(frame_pipe[0]);
      close_fd(frame_pipe[1]);

      const std::string width_arg = std::to_string(width);
      const std::string height_arg = std::to_string(height);
      const std::string camera_arg = std::to_string(camera_index(camera_device));
      const char * python = std::getenv("PICAMERA2_PYTHON");
      if (python == nullptr || python[0] == '\0') {
        python = "python3";
      }

      execlp(
        python, python, "-u", "-c", picamera2_script, width_arg.c_str(), height_arg.c_str(),
        camera_arg.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }

    close_fd(request_pipe[0]);
    close_fd(frame_pipe[1]);
    // Parent process: keep only the pipe ends used to request and receive frames.
    request_fd = request_pipe[1];
    frame_fd = frame_pipe[0];
    picamera2_pid = pid;
    return true;
  }

  bool read_frame(std::vector<std::uint8_t> & data, std::string & error)
  {
    // Ask the helper for exactly one fresh frame, then read the expected fixed-size RGB buffer.
    const std::uint8_t request = 1;
    if (!write_all(request_fd, &request, 1)) {
      error = std::string("failed to request Picamera2 frame: ") + std::strerror(errno);
      close_picamera2();
      return false;
    }

    if (!read_all(frame_fd, data.data(), data.size())) {
      error = std::string("failed to read Picamera2 frame: ") + std::strerror(errno);
      close_picamera2();
      return false;
    }
    return true;
  }

  void close_picamera2()
  {
    close_fd(request_fd);
    close_fd(frame_fd);

    if (picamera2_pid > 0) {
      int status = 0;
      waitpid(picamera2_pid, &status, 0);
      picamera2_pid = -1;
    }
  }

  static void close_fd(int & fd)
  {
    if (fd >= 0) {
      close(fd);
      fd = -1;
    }
  }

  static bool write_all(int fd, const std::uint8_t * data, std::size_t size)
  {
    // POSIX read/write can transfer partial buffers, so loop until the full frame is moved.
    std::size_t written = 0;
    while (written < size) {
      const ssize_t result = write(fd, data + written, size - written);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      if (result == 0) {
        return false;
      }
      written += static_cast<std::size_t>(result);
    }
    return true;
  }

  static bool read_all(int fd, std::uint8_t * data, std::size_t size)
  {
    std::size_t read_bytes = 0;
    while (read_bytes < size) {
      const ssize_t result = read(fd, data + read_bytes, size - read_bytes);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      if (result == 0) {
        return false;
      }
      read_bytes += static_cast<std::size_t>(result);
    }
    return true;
  }

  static int camera_index(const std::string & camera_device)
  {
    // Accept "0", "/dev/video0", or similar device strings and pass Picamera2 a numeric index.
    if (camera_device.empty()) {
      return 0;
    }

    const char * begin = camera_device.c_str();
    char * end = nullptr;
    const long parsed = std::strtol(begin, &end, 10);
    if (end != begin && *end == '\0' && parsed >= 0) {
      return static_cast<int>(parsed);
    }

    std::size_t pos = camera_device.size();
    while (pos > 0 && camera_device[pos - 1] >= '0' && camera_device[pos - 1] <= '9') {
      --pos;
    }
    if (pos < camera_device.size()) {
      return std::atoi(camera_device.c_str() + pos);
    }
    return 0;
  }

  static constexpr const char * picamera2_script =
    "import sys\n"
    "from picamera2 import Picamera2\n"
    "w=int(sys.argv[1]); h=int(sys.argv[2]); camera_num=int(sys.argv[3])\n"
    "picam2=Picamera2(camera_num=camera_num)\n"
    "cfg=picam2.create_video_configuration(main={'size':(w,h),'format':'RGB888'},buffer_count=2)\n"
    "picam2.configure(cfg)\n"
    "picam2.start()\n"
    "stdin=sys.stdin.buffer; stdout=sys.stdout.buffer\n"
    "while stdin.read(1):\n"
    "    frame=picam2.capture_array('main')\n"
    "    stdout.write(frame.tobytes())\n"
    "    stdout.flush()\n";

  pid_t picamera2_pid = -1;
  int request_fd = -1;
  int frame_fd = -1;
#endif
};

CameraCapture::CameraCapture(
  int frame_width,
  int frame_height,
  const std::string & camera_device)
: frame_width_(frame_width),
  frame_height_(frame_height),
  camera_device_(camera_device),
  impl_(std::make_unique<Impl>())
{
}

CameraCapture::~CameraCapture() = default;

bool CameraCapture::capture_frame(Frame & frame)
{
#ifdef SPRAY_VISION_USE_OPENCV
  if (!impl_->capture.isOpened()) {
    impl_->capture.open(camera_device_);
    if (!impl_->capture.isOpened()) {
      last_error_ = "failed to open camera device: " + camera_device_;
      return false;
    }

    impl_->capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    impl_->capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
  }

  cv::Mat image;
  if (!impl_->capture.read(image) || image.empty()) {
    last_error_ = "failed to read camera frame";
    return false;
  }

  if (image.cols != frame_width_ || image.rows != frame_height_) {
    cv::resize(image, image, cv::Size(frame_width_, frame_height_));
  }

  if (!image.isContinuous()) {
    image = image.clone();
  }

  frame.width = image.cols;
  frame.height = image.rows;
  frame.data.assign(image.data, image.data + image.total() * image.elemSize());
  last_error_.clear();
  return true;
#else
  // Picamera2 is used when the build does not enable the OpenCV V4L2 capture path.
  if (!impl_->start_picamera2(frame_width_, frame_height_, camera_device_, last_error_)) {
    return false;
  }

  frame.width = frame_width_;
  frame.height = frame_height_;
  // RGB888 means three bytes per pixel: red, green, blue.
  frame.data.resize(static_cast<std::size_t>(frame_width_) * static_cast<std::size_t>(frame_height_) * 3U);
  if (!impl_->read_frame(frame.data, last_error_)) {
    return false;
  }

  last_error_.clear();
  return true;
#endif
}

const std::string & CameraCapture::last_error() const
{
  return last_error_;
}

}  // namespace spray_vision
