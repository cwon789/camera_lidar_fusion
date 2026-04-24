#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#include "lidar_camera_colorizer/calibration.hpp"

namespace lidar_camera_colorizer
{
namespace
{

struct PointXYZ
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct ImageSnapshot
{
  cv::Mat image;
  rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
  bool has_stamp{false};
};

struct PointFieldView
{
  std::size_t offset{0U};
  uint8_t datatype{sensor_msgs::msg::PointField::FLOAT32};
};

template<typename T>
T read_scalar(const uint8_t * data, bool swap_endian)
{
  T value{};
  if (!swap_endian) {
    std::memcpy(&value, data, sizeof(T));
    return value;
  }

  uint8_t reversed[sizeof(T)];
  for (std::size_t index = 0; index < sizeof(T); ++index) {
    reversed[index] = data[sizeof(T) - 1U - index];
  }
  std::memcpy(&value, reversed, sizeof(T));
  return value;
}

double read_field_as_double(
  const uint8_t * point_data,
  const PointFieldView & field,
  bool is_bigendian)
{
  const uint8_t * data = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(*reinterpret_cast<const int8_t *>(data));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(*reinterpret_cast<const uint8_t *>(data));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(read_scalar<int16_t>(data, is_bigendian));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(read_scalar<uint16_t>(data, is_bigendian));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(read_scalar<int32_t>(data, is_bigendian));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(read_scalar<uint32_t>(data, is_bigendian));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(read_scalar<float>(data, is_bigendian));
    case sensor_msgs::msg::PointField::FLOAT64:
      return read_scalar<double>(data, is_bigendian);
    default:
      throw std::runtime_error("Unsupported PointCloud2 field datatype");
  }
}

bool find_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name,
  PointFieldView & view)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      view.offset = field.offset;
      view.datatype = field.datatype;
      return true;
    }
  }
  return false;
}

float pack_rgb_as_float(uint8_t red, uint8_t green, uint8_t blue)
{
  union
  {
    uint32_t rgb;
    float rgb_float;
  } packed{};
  packed.rgb =
    (static_cast<uint32_t>(red) << 16U) |
    (static_cast<uint32_t>(green) << 8U) |
    static_cast<uint32_t>(blue);
  return packed.rgb_float;
}

bool has_standard_xyz_layout(
  const PointFieldView & x_field,
  const PointFieldView & y_field,
  const PointFieldView & z_field,
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  return
    !cloud.is_bigendian &&
    x_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
    y_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
    z_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
    x_field.offset == 0U &&
    y_field.offset == 4U &&
    z_field.offset == 8U &&
    cloud.point_step >= 12U;
}

PointXYZ read_standard_xyz(const uint8_t * point_data)
{
  PointXYZ point;
  std::memcpy(&point, point_data, sizeof(PointXYZ));
  return point;
}

PointXYZ read_generic_xyz(
  const uint8_t * point_data,
  const PointFieldView & x_field,
  const PointFieldView & y_field,
  const PointFieldView & z_field,
  bool is_bigendian)
{
  return PointXYZ{
    static_cast<float>(read_field_as_double(point_data, x_field, is_bigendian)),
    static_cast<float>(read_field_as_double(point_data, y_field, is_bigendian)),
    static_cast<float>(read_field_as_double(point_data, z_field, is_bigendian))};
}

sensor_msgs::msg::PointField make_point_field(
  const std::string & name,
  uint32_t offset,
  uint8_t datatype,
  uint32_t count)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

sensor_msgs::msg::PointCloud2 make_xyzrgb_cloud_with_capacity(
  const std_msgs::msg::Header & header,
  std::size_t capacity)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1U;
  cloud.width = static_cast<uint32_t>(capacity);
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.point_step = 16U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.reserve(4U);
  cloud.fields.push_back(make_point_field("x", 0U, sensor_msgs::msg::PointField::FLOAT32, 1U));
  cloud.fields.push_back(make_point_field("y", 4U, sensor_msgs::msg::PointField::FLOAT32, 1U));
  cloud.fields.push_back(make_point_field("z", 8U, sensor_msgs::msg::PointField::FLOAT32, 1U));
  cloud.fields.push_back(make_point_field("rgb", 12U, sensor_msgs::msg::PointField::FLOAT32, 1U));
  cloud.data.resize(capacity * cloud.point_step);
  return cloud;
}

void write_xyzrgb_point(
  sensor_msgs::msg::PointCloud2 & cloud,
  std::size_t output_index,
  const PointXYZ & point,
  float rgb)
{
  uint8_t * out = &cloud.data[output_index * cloud.point_step];
  std::memcpy(out + 0U, &point.x, sizeof(float));
  std::memcpy(out + 4U, &point.y, sizeof(float));
  std::memcpy(out + 8U, &point.z, sizeof(float));
  std::memcpy(out + 12U, &rgb, sizeof(float));
}

void finalize_xyzrgb_cloud(sensor_msgs::msg::PointCloud2 & cloud, std::size_t output_count)
{
  cloud.width = static_cast<uint32_t>(output_count);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(output_count * cloud.point_step);
}

}  // namespace

class LidarCameraColorizer : public rclcpp::Node
{
public:
  LidarCameraColorizer()
  : Node("colorize_node")
  {
    declare_parameter<std::string>(
      "calib_path",
      "/home/jay/catkin_depth/src/livox_preprocessed/calib.json");
    declare_parameter<std::string>("points_topic", "");
    declare_parameter<std::string>("image_topic", "");
    declare_parameter<std::string>("output_topic", "/livox/color_points");
    declare_parameter<std::string>("lidar_frame_id", "livox_frame");
    declare_parameter<std::string>("camera_frame_id", "camera");
    declare_parameter<bool>("use_inverse_transform", true);
    declare_parameter<bool>("keep_uncolored", false);
    declare_parameter<int>("max_points", 250000);
    declare_parameter<double>("max_image_age_sec", 0.0);
    declare_parameter<double>("min_depth_m", 0.05);
    declare_parameter<bool>("output_in_camera_frame", false);

    const std::string calib_path = get_parameter("calib_path").as_string();
    calibration_ = load_calibration(calib_path);

    points_topic_ = get_parameter("points_topic").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    const auto lidar_frame_id = get_parameter("lidar_frame_id").as_string();
    camera_frame_id_ = get_parameter("camera_frame_id").as_string();
    use_inverse_transform_ = get_parameter("use_inverse_transform").as_bool();
    keep_uncolored_ = get_parameter("keep_uncolored").as_bool();
    max_points_ = get_parameter("max_points").as_int();
    max_image_age_sec_ = get_parameter("max_image_age_sec").as_double();
    min_depth_m_ = static_cast<float>(get_parameter("min_depth_m").as_double());
    output_in_camera_frame_ = get_parameter("output_in_camera_frame").as_bool();
    (void)lidar_frame_id;

    if (points_topic_.empty()) {
      points_topic_ = calibration_.points_topic;
    }
    if (image_topic_.empty()) {
      image_topic_ = calibration_.image_topic;
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarCameraColorizer::image_callback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarCameraColorizer::cloud_callback, this, std::placeholders::_1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Colorizing %s with %s; publishing %s",
      points_topic_.c_str(),
      image_topic_.c_str(),
      output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Calibration loaded from %s; use_inverse_transform=%s",
      calib_path.c_str(),
      use_inverse_transform_ ? "true" : "false");
  }

private:
  ImageSnapshot convert_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
  {
    ImageSnapshot snapshot;
    snapshot.has_stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0);
    if (snapshot.has_stamp) {
      snapshot.stamp = rclcpp::Time(msg->header.stamp);
    }

    const std::string & encoding = msg->encoding;
    if (encoding == sensor_msgs::image_encodings::MONO8) {
      snapshot.image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
      return snapshot;
    }

    if (encoding == sensor_msgs::image_encodings::MONO16) {
      cv::Mat mono16 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
      mono16.convertTo(snapshot.image, CV_8UC1, 1.0 / 256.0);
      return snapshot;
    }

    if (encoding == sensor_msgs::image_encodings::RGB8) {
      snapshot.image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
      return snapshot;
    }

    if (encoding == sensor_msgs::image_encodings::BGR8) {
      cv::Mat bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::cvtColor(bgr, snapshot.image, cv::COLOR_BGR2RGB);
      return snapshot;
    }

    if (encoding == sensor_msgs::image_encodings::RGBA8) {
      cv::Mat rgba = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8)->image;
      cv::cvtColor(rgba, snapshot.image, cv::COLOR_RGBA2RGB);
      return snapshot;
    }

    if (encoding == sensor_msgs::image_encodings::BGRA8) {
      cv::Mat bgra = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8)->image;
      cv::cvtColor(bgra, snapshot.image, cv::COLOR_BGRA2RGB);
      return snapshot;
    }

    throw std::runtime_error("Unsupported image encoding: " + encoding);
  }

  bool project_point(const cv::Vec3f & point_camera, int & u, int & v, int width, int height) const
  {
    if (point_camera[2] <= min_depth_m_) {
      return false;
    }

    const float x = point_camera[0] / point_camera[2];
    const float y = point_camera[1] / point_camera[2];
    const float k1 = calibration_.distortion[0];
    const float k2 = calibration_.distortion[1];
    const float p1 = calibration_.distortion[2];
    const float p2 = calibration_.distortion[3];
    const float k3 = calibration_.distortion[4];

    const float r2 = x * x + y * y;
    const float radial = 1.0F + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const float xd = x * radial + 2.0F * p1 * x * y + p2 * (r2 + 2.0F * x * x);
    const float yd = y * radial + p1 * (r2 + 2.0F * y * y) + 2.0F * p2 * x * y;

    u = static_cast<int>(std::lround(calibration_.fx * xd + calibration_.cx));
    v = static_cast<int>(std::lround(calibration_.fy * yd + calibration_.cy));
    return u >= 0 && u < width && v >= 0 && v < height;
  }

  cv::Vec3f transform_point(const PointXYZ & point) const
  {
    const cv::Vec3f lidar_point(point.x, point.y, point.z);
    if (use_inverse_transform_) {
      return calibration_.rotation.t() * (lidar_point - calibration_.translation);
    }
    return calibration_.rotation * lidar_point + calibration_.translation;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      ImageSnapshot snapshot = convert_image(msg);
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_ = std::move(snapshot);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "%s",
        error.what());
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    ImageSnapshot image_snapshot;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      image_snapshot = latest_image_;
    }

    if (image_snapshot.image.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for a camera image before colorizing point clouds");
      return;
    }

    const bool cloud_has_stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0);
    if (
      max_image_age_sec_ > 0.0 && cloud_has_stamp && image_snapshot.has_stamp)
    {
      const rclcpp::Duration age = rclcpp::Time(msg->header.stamp) - image_snapshot.stamp;
      const double age_seconds = std::abs(age.seconds());
      if (age_seconds > max_image_age_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Latest image is %.3fs away from cloud stamp; skipping cloud",
          age_seconds);
        return;
      }
    }

    PointFieldView x_field;
    PointFieldView y_field;
    PointFieldView z_field;
    if (!find_field(*msg, "x", x_field) || !find_field(*msg, "y", y_field) || !find_field(*msg, "z", z_field)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "PointCloud2 must contain x, y, z fields");
      return;
    }

    const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
    std::size_t stride = 1U;
    if (max_points_ > 0 && point_count > static_cast<std::size_t>(max_points_)) {
      stride = static_cast<std::size_t>(
        std::ceil(static_cast<double>(point_count) / static_cast<double>(max_points_)));
    }

    std_msgs::msg::Header header = msg->header;
    if (output_in_camera_frame_) {
      header.frame_id = camera_frame_id_;
    }

    sensor_msgs::msg::PointCloud2 output_cloud = make_xyzrgb_cloud_with_capacity(
      header, (point_count + stride - 1U) / stride);
    const bool swap_endian = msg->is_bigendian;
    const bool use_fast_path = has_standard_xyz_layout(x_field, y_field, z_field, *msg);
    std::size_t output_count = 0U;

    for (std::size_t index = 0; index < point_count; ++index) {
      if ((index % stride) != 0U) {
        continue;
      }

      const uint8_t * point_data = &msg->data[index * msg->point_step];
      PointXYZ lidar_point;
      try {
        lidar_point = use_fast_path ?
          read_standard_xyz(point_data) :
          read_generic_xyz(point_data, x_field, y_field, z_field, swap_endian);
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Failed to parse PointCloud2: %s",
          error.what());
        return;
      }

      if (
        !std::isfinite(lidar_point.x) ||
        !std::isfinite(lidar_point.y) ||
        !std::isfinite(lidar_point.z))
      {
        continue;
      }
      const cv::Vec3f camera_point = transform_point(lidar_point);

      int u = 0;
      int v = 0;
      const bool valid_projection = project_point(
        camera_point,
        u,
        v,
        image_snapshot.image.cols,
        image_snapshot.image.rows);

      uint8_t red = 128U;
      uint8_t green = 128U;
      uint8_t blue = 128U;
      if (valid_projection) {
        if (image_snapshot.image.type() == CV_8UC1) {
          const uint8_t gray = image_snapshot.image.at<uint8_t>(v, u);
          red = gray;
          green = gray;
          blue = gray;
        } else {
          const cv::Vec3b color = image_snapshot.image.at<cv::Vec3b>(v, u);
          red = color[0];
          green = color[1];
          blue = color[2];
        }
      } else if (!keep_uncolored_) {
        continue;
      }

      const PointXYZ output_point = output_in_camera_frame_ ?
        PointXYZ{camera_point[0], camera_point[1], camera_point[2]} :
        lidar_point;
      write_xyzrgb_point(
        output_cloud,
        output_count,
        output_point,
        pack_rgb_as_float(red, green, blue));
      ++output_count;
    }

    if (output_count == 0U) {
      if (!keep_uncolored_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "No LiDAR points projected into the current image");
      }
      return;
    }

    finalize_xyzrgb_cloud(output_cloud, output_count);
    cloud_pub_->publish(output_cloud);
  }

  CalibrationData calibration_;
  std::mutex image_mutex_;
  ImageSnapshot latest_image_;

  std::string points_topic_;
  std::string image_topic_;
  std::string output_topic_;
  std::string camera_frame_id_;
  bool use_inverse_transform_{true};
  bool keep_uncolored_{false};
  int max_points_{0};
  double max_image_age_sec_{0.0};
  float min_depth_m_{0.05F};
  bool output_in_camera_frame_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace lidar_camera_colorizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_camera_colorizer::LidarCameraColorizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
