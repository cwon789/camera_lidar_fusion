#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include "lidar_camera_colorizer/calibration.hpp"

namespace lidar_camera_colorizer
{
namespace
{

struct DemoPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct PlyProperty
{
  std::string name;
  std::size_t offset{0U};
  std::size_t size{0U};
  std::string type;
};

std::size_t ply_type_size(const std::string & type)
{
  if (type == "char" || type == "uchar" || type == "int8" || type == "uint8") {
    return 1U;
  }
  if (type == "short" || type == "ushort" || type == "int16" || type == "uint16") {
    return 2U;
  }
  if (type == "int" || type == "uint" || type == "int32" || type == "uint32" || type == "float" ||
    type == "float32")
  {
    return 4U;
  }
  if (type == "double" || type == "float64") {
    return 8U;
  }
  throw std::runtime_error("Unsupported PLY property type: " + type);
}

template<typename T>
T read_binary_little_endian(const uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

float ply_property_to_float(const uint8_t * data, const PlyProperty & property)
{
  const uint8_t * field = data + property.offset;
  if (property.type == "char" || property.type == "int8") {
    return static_cast<float>(read_binary_little_endian<int8_t>(field));
  }
  if (property.type == "uchar" || property.type == "uint8") {
    return static_cast<float>(read_binary_little_endian<uint8_t>(field));
  }
  if (property.type == "short" || property.type == "int16") {
    return static_cast<float>(read_binary_little_endian<int16_t>(field));
  }
  if (property.type == "ushort" || property.type == "uint16") {
    return static_cast<float>(read_binary_little_endian<uint16_t>(field));
  }
  if (property.type == "int" || property.type == "int32") {
    return static_cast<float>(read_binary_little_endian<int32_t>(field));
  }
  if (property.type == "uint" || property.type == "uint32") {
    return static_cast<float>(read_binary_little_endian<uint32_t>(field));
  }
  if (property.type == "float" || property.type == "float32") {
    return read_binary_little_endian<float>(field);
  }
  if (property.type == "double" || property.type == "float64") {
    return static_cast<float>(read_binary_little_endian<double>(field));
  }
  throw std::runtime_error("Unsupported PLY property type: " + property.type);
}

std::vector<DemoPoint> read_binary_little_endian_ply(const std::string & path, int point_limit)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to open PLY file: " + path);
  }

  std::string line;
  std::size_t vertex_count = 0U;
  bool in_vertex_section = false;
  std::vector<PlyProperty> properties;
  std::size_t record_size = 0U;
  bool found_format = false;

  while (std::getline(stream, line)) {
    if (line == "format binary_little_endian 1.0") {
      found_format = true;
      continue;
    }

    if (line.rfind("element ", 0) == 0) {
      std::istringstream line_stream(line);
      std::string element_keyword;
      std::string element_name;
      line_stream >> element_keyword >> element_name;
      in_vertex_section = (element_name == "vertex");
      if (in_vertex_section) {
        line_stream >> vertex_count;
      }
      continue;
    }

    if (line.rfind("property ", 0) == 0 && in_vertex_section) {
      std::istringstream line_stream(line);
      std::string property_keyword;
      std::string property_type;
      std::string property_name;
      line_stream >> property_keyword >> property_type >> property_name;
      if (property_keyword != "property" || property_type == "list") {
        throw std::runtime_error("PLY list properties are not supported");
      }
      const std::size_t size = ply_type_size(property_type);
      properties.push_back(PlyProperty{property_name, record_size, size, property_type});
      record_size += size;
      continue;
    }

    if (line == "end_header") {
      break;
    }
  }

  if (!found_format) {
    throw std::runtime_error("Only binary_little_endian PLY files are supported");
  }
  if (vertex_count == 0U || properties.empty()) {
    throw std::runtime_error("PLY file does not declare a valid vertex element");
  }

  const auto find_property = [&properties](const std::string & name) -> const PlyProperty * {
      for (const auto & property : properties) {
        if (property.name == name) {
          return &property;
        }
      }
      return nullptr;
    };

  const PlyProperty * x_property = find_property("x");
  const PlyProperty * y_property = find_property("y");
  const PlyProperty * z_property = find_property("z");
  const PlyProperty * intensity_property = find_property("intensity");
  if (x_property == nullptr || y_property == nullptr || z_property == nullptr) {
    throw std::runtime_error("PLY file must contain x, y, z vertex properties");
  }

  std::size_t stride = 1U;
  if (point_limit > 0 && vertex_count > static_cast<std::size_t>(point_limit)) {
    stride = static_cast<std::size_t>(
      std::ceil(static_cast<double>(vertex_count) / static_cast<double>(point_limit)));
  }

  std::vector<DemoPoint> points;
  points.reserve((vertex_count + stride - 1U) / stride);

  std::vector<uint8_t> record(record_size);
  for (std::size_t index = 0; index < vertex_count; ++index) {
    stream.read(reinterpret_cast<char *>(record.data()), static_cast<std::streamsize>(record.size()));
    if (!stream) {
      throw std::runtime_error("PLY data ended unexpectedly");
    }

    if ((index % stride) != 0U) {
      continue;
    }

    DemoPoint point;
    point.x = ply_property_to_float(record.data(), *x_property);
    point.y = ply_property_to_float(record.data(), *y_property);
    point.z = ply_property_to_float(record.data(), *z_property);
    if (intensity_property != nullptr) {
      point.intensity = ply_property_to_float(record.data(), *intensity_property);
    }
    points.push_back(point);
  }

  return points;
}

sensor_msgs::msg::PointCloud2 make_cloud_msg(
  const std_msgs::msg::Header & header,
  const std::vector<DemoPoint> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1U;
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> out_intensity(cloud, "intensity");

  for (std::size_t index = 0; index < points.size(); ++index, ++out_x, ++out_y, ++out_z, ++out_intensity) {
    *out_x = points[index].x;
    *out_y = points[index].y;
    *out_z = points[index].z;
    *out_intensity = points[index].intensity;
  }

  return cloud;
}

sensor_msgs::msg::Image make_image_msg(const std_msgs::msg::Header & header, const cv::Mat & image)
{
  sensor_msgs::msg::Image message;
  message.header = header;
  message.height = static_cast<uint32_t>(image.rows);
  message.width = static_cast<uint32_t>(image.cols);
  message.is_bigendian = false;

  if (image.type() == CV_8UC1) {
    message.encoding = sensor_msgs::image_encodings::MONO8;
    message.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  } else if (image.type() == CV_8UC3) {
    message.encoding = sensor_msgs::image_encodings::BGR8;
    message.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  } else {
    throw std::runtime_error("Unsupported image type for demo publisher");
  }

  const auto * begin = image.ptr<uint8_t>(0);
  const auto * end = begin + (image.rows * static_cast<int>(image.step));
  message.data.assign(begin, end);
  return message;
}

}  // namespace

class DemoPublisher : public rclcpp::Node
{
public:
  DemoPublisher()
  : Node("demo_publisher")
  {
    declare_parameter<std::string>(
      "calib_path",
      "/home/jay/catkin_depth/src/livox_preprocessed/calib.json");
    declare_parameter<std::string>(
      "data_dir",
      "/home/jay/catkin_depth/src/livox_preprocessed");
    declare_parameter<std::string>("sample_name", "rosbag2_2023_03_09-13_42_46");
    declare_parameter<std::string>("frame_id", "livox_frame");
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<int>("point_limit", 250000);
    declare_parameter<std::string>("points_topic", "");
    declare_parameter<std::string>("image_topic", "");

    const std::string calib_path = get_parameter("calib_path").as_string();
    const CalibrationData calibration = load_calibration(calib_path);

    points_topic_ = get_parameter("points_topic").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    if (points_topic_.empty()) {
      points_topic_ = calibration.points_topic;
    }
    if (image_topic_.empty()) {
      image_topic_ = calibration.image_topic;
    }

    const std::string data_dir = get_parameter("data_dir").as_string();
    const std::string sample_name = get_parameter("sample_name").as_string();
    const int point_limit = get_parameter("point_limit").as_int();

    const std::string ply_path = data_dir + "/" + sample_name + ".ply";
    const std::string image_path = data_dir + "/" + sample_name + ".png";

    cloud_points_ = read_binary_little_endian_ply(ply_path, point_limit);
    image_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
    if (image_.empty()) {
      throw std::runtime_error("Failed to read image: " + image_path);
    }
    if (image_.type() != CV_8UC1 && image_.type() != CV_8UC3) {
      throw std::runtime_error("Demo image must be mono8 or bgr8 compatible");
    }

    frame_id_ = get_parameter("frame_id").as_string();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic_, rclcpp::SensorDataQoS());
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());

    const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DemoPublisher::publish_once, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing demo sample %s: %zu points, image %dx%d",
      sample_name.c_str(),
      cloud_points_.size(),
      image_.cols,
      image_.rows);
    RCLCPP_INFO(
      get_logger(),
      "Publishing %s and %s",
      points_topic_.c_str(),
      image_topic_.c_str());
  }

private:
  void publish_once()
  {
    const auto stamp = now();

    std_msgs::msg::Header image_header;
    image_header.stamp = stamp;
    image_header.frame_id = "camera";

    std_msgs::msg::Header cloud_header;
    cloud_header.stamp = stamp;
    cloud_header.frame_id = frame_id_;

    image_pub_->publish(make_image_msg(image_header, image_));
    cloud_pub_->publish(make_cloud_msg(cloud_header, cloud_points_));
  }

  std::vector<DemoPoint> cloud_points_;
  cv::Mat image_;
  std::string frame_id_;
  std::string points_topic_;
  std::string image_topic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lidar_camera_colorizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_camera_colorizer::DemoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
