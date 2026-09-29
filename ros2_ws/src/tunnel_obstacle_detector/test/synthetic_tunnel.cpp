#include "synthetic_tunnel.hpp"

#include <cmath>
#include <limits>
#include <random>

namespace tod_test
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

std::vector<double> elevationTable()
{
  std::vector<double> e;
  for (int i = 0; i < 26; ++i) {
    e.push_back(14.5 - 0.5 * i);  // +14.5 .. +2.0
  }
  for (int i = 0; i < 64; ++i) {
    e.push_back(1.875 - 0.125 * i);  // +1.875 .. -6.0
  }
  for (int i = 0; i < 38; ++i) {
    e.push_back(-6.5 - 0.5 * i);  // -6.5 .. -25.0
  }
  return e;
}

/// smallest positive root of a t^2 + b t + c = 0
double smallestPositiveRoot(double a, double b, double c)
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  if (std::abs(a) < 1e-12) {
    if (std::abs(b) < 1e-12) {
      return inf;
    }
    const double t = -c / b;
    return t > 1e-6 ? t : inf;
  }
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return inf;
  }
  const double s = std::sqrt(disc);
  double t1 = (-b - s) / (2.0 * a);
  double t2 = (-b + s) / (2.0 * a);
  if (t1 > t2) {
    std::swap(t1, t2);
  }
  if (t1 > 1e-6) {
    return t1;
  }
  return t2 > 1e-6 ? t2 : inf;
}

struct Hit
{
  double t = std::numeric_limits<double>::infinity();
};

tod::TrackCloud render(const TunnelSpec & s, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, s.noise);
  const std::vector<double> elevations = elevationTable();
  const auto axis = [&s](double x) {return 0.5 * s.curvature * x * x;};
  tod::TrackCloud cloud;
  const double oy = s.sensor_lateral;
  const double oz = s.sensor_height;

  const int n_az = static_cast<int>(std::lround(2.0 * s.azimuth_half_fov_deg / s.azimuth_step_deg)) + 1;
  for (int a = 0; a < n_az; ++a) {
    const double az = (s.azimuth_half_fov_deg - a * s.azimuth_step_deg) * kDegToRad;  // left positive
    for (std::size_t ring = 0; ring < elevations.size(); ++ring) {
      const double el = elevations[ring] * kDegToRad;
      const double dx = std::cos(el) * std::cos(az);
      const double dy = std::cos(el) * std::sin(az);
      const double dz = std::sin(el);
      double best = std::numeric_limits<double>::infinity();

      if (dz < 0.0) {
        const double tf = -oz / dz;  // floor
        best = std::min(best, tf);
        const double tr = (s.rail_height - oz) / dz;  // rail heads
        const double xr = tr * dx;
        const double yr = oy + tr * dy;
        if (xr > 0.0 && (std::abs(yr - axis(xr) - s.gauge_half) < 0.036 || std::abs(yr - axis(xr) + s.gauge_half) < 0.036)) {
          best = std::min(best, tr);
        }
      } else if (dz > 0.0) {
        best = std::min(best, (s.ceiling - oz) / dz);
      }
      for (double side : {1.0, -1.0}) {  // curved walls: oy + t dy - k (t dx)^2 / 2 = side * w
        const double tw = smallestPositiveRoot(-0.5 * s.curvature * dx * dx, dy, oy - side * s.wall_half_width);
        best = std::min(best, tw);
      }
      for (const Box & b : s.boxes) {  // slab test, box lateral measured from the axis at its near face
        const double yc = axis(b.x_near) + b.lateral;
        const double lo[3] = {b.x_near, yc - 0.5 * b.width, 0.0};
        const double hi[3] = {b.x_near + b.length, yc + 0.5 * b.width, b.height};
        const double o[3] = {0.0, oy, oz};
        const double d[3] = {dx, dy, dz};
        double tmin = 0.0;
        double tmax = std::numeric_limits<double>::infinity();
        bool ok = true;
        for (int k = 0; k < 3 && ok; ++k) {
          if (std::abs(d[k]) < 1e-12) {
            ok = o[k] >= lo[k] && o[k] <= hi[k];
            continue;
          }
          double t1 = (lo[k] - o[k]) / d[k];
          double t2 = (hi[k] - o[k]) / d[k];
          if (t1 > t2) {
            std::swap(t1, t2);
          }
          tmin = std::max(tmin, t1);
          tmax = std::min(tmax, t2);
          ok = tmin <= tmax;
        }
        if (ok && tmin > 1e-6) {
          best = std::min(best, tmin);
        }
      }
      if (!std::isfinite(best) || best > s.max_range || best < 1.0) {
        continue;
      }
      const double r = best + noise(rng);
      tod::TrackPoint p;
      p.x = static_cast<float>(r * dx);
      p.y = static_cast<float>(oy + r * dy);
      p.z = static_cast<float>(oz + r * dz);
      p.ring = static_cast<uint16_t>(ring);
      p.raw_index = static_cast<uint32_t>(cloud.size());
      cloud.push_back(p);
    }
  }
  return cloud;
}
}  // namespace

tod::TrackCloud renderTunnelTrackFrame(const TunnelSpec & spec, unsigned seed)
{
  return render(spec, seed);
}

std::vector<tod::RawPoint> renderTunnel(const TunnelSpec & spec, unsigned seed)
{
  const tod::TrackCloud track = render(spec, seed);
  std::vector<tod::RawPoint> raw;
  raw.reserve(track.size());
  for (const auto & p : track) {
    tod::RawPoint q;
    // track frame has its origin on the bed below the lidar; the cloud frame is centred on the lidar
    q.x = p.y - static_cast<float>(spec.sensor_lateral);
    q.y = -p.x;
    q.z = p.z - static_cast<float>(spec.sensor_height);
    q.intensity = 20.0F;
    q.ring = p.ring;
    raw.push_back(q);
  }
  return raw;
}

}  // namespace tod_test
