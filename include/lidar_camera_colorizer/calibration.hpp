#ifndef LIDAR_CAMERA_COLORIZER__CALIBRATION_HPP_
#define LIDAR_CAMERA_COLORIZER__CALIBRATION_HPP_

#include <array>
#include <string>

#include <opencv2/core.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace lidar_camera_colorizer
{

struct CalibrationData
{
  float fx{0.0F};
  float fy{0.0F};
  float cx{0.0F};
  float cy{0.0F};
  std::array<float, 5> distortion{{0.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
  cv::Vec3f translation{0.0F, 0.0F, 0.0F};
  cv::Matx33f rotation = cv::Matx33f::eye();
  std::string points_topic{"/livox/points"};
  std::string image_topic{"/image"};
};

CalibrationData load_calibration(const std::string & calib_path);
tf2::Transform load_tf_transform(const std::string & calib_path, bool use_inverse_transform);

}  // namespace lidar_camera_colorizer

#endif  // LIDAR_CAMERA_COLORIZER__CALIBRATION_HPP_
