#include "tunnel_obstacle_detector/ros/cloud_conversion.hpp"

#include <cstring>

namespace tod_ros
{

namespace
{
using sensor_msgs::msg::PointField;

inline double readAs(const uint8_t * data, uint8_t type)
{
  switch (type) {
    case PointField::INT8: {int8_t v; std::memcpy(&v, data, 1); return v;}
    case PointField::UINT8: {uint8_t v; std::memcpy(&v, data, 1); return v;}
    case PointField::INT16: {int16_t v; std::memcpy(&v, data, 2); return v;}
    case PointField::UINT16: {uint16_t v; std::memcpy(&v, data, 2); return v;}
    case PointField::INT32: {int32_t v; std::memcpy(&v, data, 4); return v;}
    case PointField::UINT32: {uint32_t v; std::memcpy(&v, data, 4); return v;}
    case PointField::FLOAT32: {float v; std::memcpy(&v, data, 4); return v;}
    case PointField::FLOAT64: {double v; std::memcpy(&v, data, 8); return v;}
    default: return 0.0;
  }
}
}  // namespace

CloudLayout analyzeLayout(const sensor_msgs::msg::PointCloud2 & msg, std::string & error)
{
  CloudLayout layout;
  layout.point_step = msg.point_step;
  if (msg.is_bigendian) {
    error = "big-endian point clouds are not supported";
    return layout;
  }
  for (const auto & f : msg.fields) {
    if (f.name == "x") {
      layout.x = static_cast<int>(f.offset);
      layout.xyz_type = f.datatype;
    } else if (f.name == "y") {
      layout.y = static_cast<int>(f.offset);
    } else if (f.name == "z") {
      layout.z = static_cast<int>(f.offset);
    } else if (f.name == "intensity" || f.name == "reflectivity") {
      if (layout.intensity < 0 || f.name == "intensity") {
        layout.intensity = static_cast<int>(f.offset);
        layout.intensity_type = f.datatype;
      }
    } else if (f.name == "ring" || f.name == "channel" || f.name == "laser_id") {
      layout.ring = static_cast<int>(f.offset);
      layout.ring_type = f.datatype;
    }
  }
  if (layout.x < 0 || layout.y < 0 || layout.z < 0) {
    error = "point cloud has no x/y/z fields";
    return layout;
  }
  if (layout.xyz_type != PointField::FLOAT32 && layout.xyz_type != PointField::FLOAT64) {
    error = "x/y/z must be FLOAT32 or FLOAT64";
    return layout;
  }
  layout.valid = true;
  return layout;
}

void convertCloud(
  const sensor_msgs::msg::PointCloud2 & msg, const CloudLayout & layout, std::vector<tod::RawPoint> & out)
{
  const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
  out.resize(n);
  const uint8_t * data = msg.data.data();
  const bool fast_xyz = layout.xyz_type == PointField::FLOAT32;
  for (std::size_t i = 0; i < n; ++i) {
    const uint8_t * pt = data + i * layout.point_step;
    tod::RawPoint & rp = out[i];
    if (fast_xyz) {
      std::memcpy(&rp.x, pt + layout.x, 4);
      std::memcpy(&rp.y, pt + layout.y, 4);
      std::memcpy(&rp.z, pt + layout.z, 4);
    } else {
      rp.x = static_cast<float>(readAs(pt + layout.x, layout.xyz_type));
      rp.y = static_cast<float>(readAs(pt + layout.y, layout.xyz_type));
      rp.z = static_cast<float>(readAs(pt + layout.z, layout.xyz_type));
    }
    if (layout.intensity >= 0) {
      if (layout.intensity_type == PointField::FLOAT32) {
        std::memcpy(&rp.intensity, pt + layout.intensity, 4);
      } else {
        rp.intensity = static_cast<float>(readAs(pt + layout.intensity, layout.intensity_type));
      }
    } else {
      rp.intensity = 0.0F;
    }
    if (layout.ring >= 0) {
      if (layout.ring_type == PointField::UINT16) {
        std::memcpy(&rp.ring, pt + layout.ring, 2);
      } else {
        rp.ring = static_cast<uint16_t>(readAs(pt + layout.ring, layout.ring_type));
      }
    } else {
      rp.ring = 0;
    }
  }
}

}  // namespace tod_ros
