#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "synthetic_tunnel.hpp"
#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"
#include "tunnel_obstacle_detector/core/track_advance.hpp"

namespace
{
/// A tunnel whose transverse structures (brackets beside the track) sit `shift` metres closer to the train.
tod_test::TunnelSpec tunnelAt(double shift, double curvature = 0.0, double far = 88.0)
{
  tod_test::TunnelSpec spec;
  spec.curvature = curvature;
  // a wide tunnel on purpose: a smooth wall inside the profile band would contribute only the lidar's own ring
  // pattern, which sits at fixed distances and does not move with the train. Real tunnels are not smooth - ring
  // joints, brackets and portals every few metres are exactly the signature this measures.
  spec.wall_half_width = 4.0;
  // on a curve the tunnel wall itself cuts the view off (at R ~ 670 m the ray meets it at ~73 m), so the
  // structures have to sit inside what is actually visible
  // irregular spacing on purpose: evenly spaced structures make the profile periodic, and a correlation cannot
  // tell one period from the next (a real tunnel is irregular - portals, brackets, joints, cable hangers)
  for (double x = 10.0; x < far; x += 1.3 + 1.4 * std::fmod(x * 0.37, 1.0)) {
    // a pair of wall brackets: inside the profile band (1.0-3.2 m high, within 3 m of the axis), outside the
    // clearance envelope, and short enough along the track to make a signature
    spec.boxes.push_back({x - shift, 0.4, 2.0, 0.4, 2.2});
    spec.boxes.push_back({x - shift + 0.3, 0.4, -2.0, 0.4, 2.2});
  }
  return spec;
}

/// The axis of the rendered tunnel, given exactly: this test is about the distance measurement, not about the
/// corridor search, and a hand-made corridor keeps the two failures apart.
tod::Corridor exactCorridor(double curvature)
{
  tod::Corridor c;
  c.valid = true;
  c.x_res = 0.5;
  const int cells = static_cast<int>(200.0 / c.x_res);
  c.lateral.resize(static_cast<std::size_t>(cells));
  c.bed.assign(static_cast<std::size_t>(cells), 0.0F);
  c.heading.assign(static_cast<std::size_t>(cells), 0.0F);
  c.spread.assign(static_cast<std::size_t>(cells), 0.0F);
  for (int i = 0; i < cells; ++i) {
    const double x = (i + 0.5) * c.x_res;
    c.lateral[static_cast<std::size_t>(i)] = static_cast<float>(0.5 * curvature * x * x);
  }
  return c;
}

double measure(double shift, double curvature = 0.0, double far = 88.0)
{
  tod::TrackAdvance advance;
  const auto corridor = exactCorridor(curvature);
  double last = std::nan("");
  for (double s : {0.0, shift, 2.0 * shift}) {
    const auto cloud = tod_test::renderTunnelTrackFrame(tunnelAt(s, curvature, far));
    last = advance.update(cloud, corridor, 0.1);
  }
  return last;
}
}  // namespace

TEST(TrackAdvance, MeasuresTheDrivenDistance)
{
  EXPECT_NEAR(measure(1.5), 1.5, 0.15);
}

TEST(TrackAdvance, MeasuresItOnACurveToo)
{
  const double straight_short = measure(2.3, 0.0, 60.0);
  const double curved = measure(2.3, 1.5e-3, 60.0);
  EXPECT_NEAR(straight_short, 2.3, 0.2) << "straight, structures only to 60 m";
  EXPECT_NEAR(curved, 2.3, 0.2) << "R ~ 670 m, structures only to 60 m";
}

TEST(TrackAdvance, SaysNothingWhenThereIsNoSignature)
{
  // a bare tube: the walls run along the track and repeat, so no shift can be measured from them
  tod::TrackAdvance advance;
  tod_test::TunnelSpec spec;
  const auto corridor = exactCorridor(0.0);
  double last = 0.0;
  for (int i = 0; i < 3; ++i) {
    const auto cloud = tod_test::renderTunnelTrackFrame(spec);
    last = advance.update(cloud, corridor, 0.1);
  }
  EXPECT_TRUE(std::isnan(last) || std::abs(last) < 0.3) << "reported " << last;
}
