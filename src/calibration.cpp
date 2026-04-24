#include "lidar_camera_colorizer/calibration.hpp"

#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

namespace lidar_camera_colorizer
{
namespace
{

std::string read_optional_string(const cv::FileNode & node, const std::string & fallback)
{
  if (node.empty() || !node.isString()) {
    return fallback;
  }
  return static_cast<std::string>(node);
}

tf2::Quaternion read_quaternion_xyzw(const cv::FileNode & node)
{
  if (node.size() != 7) {
    throw std::runtime_error("results.T_lidar_camera must be [tx, ty, tz, qx, qy, qz, qw]");
  }

  tf2::Quaternion quaternion(
    static_cast<double>(node[3]),
    static_cast<double>(node[4]),
    static_cast<double>(node[5]),
    static_cast<double>(node[6]));

  if (quaternion.length2() == 0.0) {
    throw std::runtime_error("Quaternion norm is zero");
  }
  quaternion.normalize();
  return quaternion;
}

cv::Vec3f read_translation(const cv::FileNode & node)
{
  if (node.size() != 7) {
    throw std::runtime_error("results.T_lidar_camera must be [tx, ty, tz, qx, qy, qz, qw]");
  }

  return cv::Vec3f(
    static_cast<float>(node[0]),
    static_cast<float>(node[1]),
    static_cast<float>(node[2]));
}

}  // namespace

CalibrationData load_calibration(const std::string & calib_path)
{
  cv::FileStorage storage(calib_path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (!storage.isOpened()) {
    throw std::runtime_error("Failed to open calibration file: " + calib_path);
  }

  const cv::FileNode camera_node = storage["camera"];
  const cv::FileNode intrinsics_node = camera_node["intrinsics"];
  if (intrinsics_node.size() != 4) {
    throw std::runtime_error("camera.intrinsics must contain [fx, fy, cx, cy]");
  }

  CalibrationData calibration;
  calibration.fx = static_cast<float>(intrinsics_node[0]);
  calibration.fy = static_cast<float>(intrinsics_node[1]);
  calibration.cx = static_cast<float>(intrinsics_node[2]);
  calibration.cy = static_cast<float>(intrinsics_node[3]);

  const cv::FileNode distortion_node = camera_node["distortion_coeffs"];
  for (int index = 0; index < 5 && index < static_cast<int>(distortion_node.size()); ++index) {
    calibration.distortion[static_cast<std::size_t>(index)] =
      static_cast<float>(distortion_node[index]);
  }

  const cv::FileNode transform_node = storage["results"]["T_lidar_camera"];
  calibration.translation = read_translation(transform_node);

  const tf2::Quaternion quaternion = read_quaternion_xyzw(transform_node);
  const tf2::Matrix3x3 rotation_matrix(quaternion);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      calibration.rotation(row, column) = static_cast<float>(rotation_matrix[row][column]);
    }
  }

  const cv::FileNode meta_node = storage["meta"];
  calibration.points_topic = read_optional_string(meta_node["points_topic"], calibration.points_topic);
  calibration.image_topic = read_optional_string(meta_node["image_topic"], calibration.image_topic);

  return calibration;
}

tf2::Transform load_tf_transform(const std::string & calib_path, bool use_inverse_transform)
{
  const CalibrationData calibration = load_calibration(calib_path);

  tf2::Matrix3x3 rotation(
    calibration.rotation(0, 0), calibration.rotation(0, 1), calibration.rotation(0, 2),
    calibration.rotation(1, 0), calibration.rotation(1, 1), calibration.rotation(1, 2),
    calibration.rotation(2, 0), calibration.rotation(2, 1), calibration.rotation(2, 2));
  tf2::Vector3 translation(
    calibration.translation[0],
    calibration.translation[1],
    calibration.translation[2]);

  tf2::Transform transform(rotation, translation);
  if (use_inverse_transform) {
    transform = transform.inverse();
  }
  return transform;
}

}  // namespace lidar_camera_colorizer
