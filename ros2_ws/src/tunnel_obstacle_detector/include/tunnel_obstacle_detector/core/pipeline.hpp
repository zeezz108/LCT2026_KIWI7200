#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"
#include "tunnel_obstacle_detector/core/ground_calibration.hpp"
#include "tunnel_obstacle_detector/core/obstacle_detector.hpp"
#include "tunnel_obstacle_detector/core/track_advance.hpp"
#include "tunnel_obstacle_detector/core/track_axis.hpp"
#include "tunnel_obstacle_detector/core/tracker.hpp"
#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

struct PipelineParams
{
  std::string forward_axis = "-y";  ///< cloud axis looking ahead of the train, or "auto"
  std::string forward_axis_fallback = "-y";  ///< used by "auto" when the first frames do not tell
  double min_range = 1.0;
  double max_range = 250.0;
  double visibility_lateral = 3.0;  ///< corridor band used to measure how far the track is observed
  bool dedupe_dual_returns = true;  ///< drop second returns identical to the first one (organized dual-return clouds)
  double virtual_ring_deg = 0.1;    ///< elevation bin that replaces the ring number when the cloud has no ring field

  GroundCalibrationParams calibration;
  TrackAxisParams axis;
  CorridorParams corridor;
  TrackAdvanceParams advance;
  DetectorParams detector;
  TrackerParams tracker;
};

struct StageTimings
{
  double transform_ms = 0.0;
  double calibration_ms = 0.0;
  double axis_ms = 0.0;
  double corridor_ms = 0.0;
  double detection_ms = 0.0;
  double tracking_ms = 0.0;
  double total_ms = 0.0;
};

/// Everything the pipeline knows about one frame.
struct FrameResult
{
  double stamp = 0.0;
  bool valid = false;          ///< calibration available and the frame was processed
  std::size_t num_points = 0;  ///< valid returns

  SensorPose pose;
  AxisModel axis;
  std::vector<AxisDetection> axis_detections;
  Corridor corridor;
  std::vector<Cluster> clusters;           ///< candidates of this frame
  std::vector<Cluster> rejected_clusters;  ///< dropped as tunnel structures
  std::vector<Track> obstacles;            ///< confirmed objects, nearest first

  Level level = Level::kClear;
  double nearest_distance = 0.0;  ///< NaN if no DANGER object
  double nearest_lateral = 0.0;
  double time_to_collision = 0.0;
  double confidence = 0.0;
  double free_distance = 0.0;     ///< observed, obstacle-free length of the corridor
  double trusted_length = 0.0;    ///< corridor length where the axis is certain enough for detection
  double advance = 0.0;           ///< metres driven since the previous frame (NaN if not measured)
  double advance_correlation = 0.0;
  double speed = 0.0;             ///< [m/s] of the train, from the lidar alone (NaN until measured)

  StageTimings timings;
};

/// Per-frame processing chain: calibration -> track axis -> corridor -> zone check -> tracking.
class Pipeline
{
public:
  explicit Pipeline(const PipelineParams & params = PipelineParams());

  const FrameResult & process(const std::vector<RawPoint> & raw, double stamp);

  const FrameResult & result() const { return result_; }
  const TrackCloud & trackCloud() const { return cloud_; }
  const std::vector<uint8_t> & zoneLabels() const { return labels_; }
  const PipelineParams & params() const { return params_; }
  const CorridorEstimator & corridorEstimator() const { return corridor_estimator_; }
  /// Forward axis in use ("" while "auto" is still undecided).
  const std::string & forwardAxis() const { return forward_axis_; }

  /// Track-frame point back to the frame of the incoming cloud.
  Eigen::Vector3d trackToCloud(const Eigen::Vector3d & track_point) const;
  /// Rotation part of trackToCloud.
  Eigen::Matrix3d trackToCloudRotation() const;

  void reset();

private:
  void updateForwardAxis(const std::vector<RawPoint> & raw);

  PipelineParams params_;
  std::string forward_axis_;
  int forward_axis_frames_ = 0;
  Eigen::Matrix3d mount_rotation_;
  GroundCalibrator calibrator_;
  TrackAxisDetector axis_detector_;
  CorridorEstimator corridor_estimator_;
  TrackAdvance advance_;
  ObstacleDetector detector_;
  Tracker tracker_;

  FrameResult result_;
  AxisModel last_axis_;
  Corridor last_corridor_;
  double last_stamp_ = 0.0;
  bool has_stamp_ = false;
  std::vector<Eigen::Vector3f> mount_points_;
  TrackCloud cloud_;
  std::vector<uint8_t> labels_;
};

}  // namespace tod
