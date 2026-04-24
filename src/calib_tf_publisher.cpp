#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "lidar_camera_colorizer/calibration.hpp"

namespace lidar_camera_colorizer
{

class CalibrationTfPublisher : public rclcpp::Node
{
public:
  CalibrationTfPublisher()
  : Node("calib_tf_publisher")
  {
    declare_parameter<std::string>(
      "calib_path",
      "/home/jay/catkin_depth/src/livox_preprocessed/calib.json");
    declare_parameter<std::string>("lidar_frame_id", "livox_frame");
    declare_parameter<std::string>("camera_frame_id", "camera");
    declare_parameter<bool>("use_inverse_transform", true);

    const std::string calib_path = get_parameter("calib_path").as_string();
    const std::string lidar_frame_id = get_parameter("lidar_frame_id").as_string();
    const std::string camera_frame_id = get_parameter("camera_frame_id").as_string();
    const bool use_inverse_transform = get_parameter("use_inverse_transform").as_bool();

    const tf2::Transform transform = load_tf_transform(calib_path, use_inverse_transform);
    const tf2::Vector3 translation = transform.getOrigin();
    const tf2::Quaternion rotation = transform.getRotation();

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = lidar_frame_id;
    message.child_frame_id = camera_frame_id;
    message.transform.translation.x = translation.x();
    message.transform.translation.y = translation.y();
    message.transform.translation.z = translation.z();
    message.transform.rotation.x = rotation.x();
    message.transform.rotation.y = rotation.y();
    message.transform.rotation.z = rotation.z();
    message.transform.rotation.w = rotation.w();
    broadcaster_->sendTransform(message);

    RCLCPP_INFO(
      get_logger(),
      "Published static TF %s -> %s from %s (use_inverse_transform=%s)",
      lidar_frame_id.c_str(),
      camera_frame_id.c_str(),
      calib_path.c_str(),
      use_inverse_transform ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "translation=(%.4f, %.4f, %.4f), quaternion=(%.4f, %.4f, %.4f, %.4f)",
      translation.x(),
      translation.y(),
      translation.z(),
      rotation.x(),
      rotation.y(),
      rotation.z(),
      rotation.w());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

}  // namespace lidar_camera_colorizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_camera_colorizer::CalibrationTfPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
