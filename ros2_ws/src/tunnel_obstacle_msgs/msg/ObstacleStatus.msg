# Detector verdict for one lidar frame.

std_msgs/Header header            # stamp and frame_id of the source point cloud

uint8 LEVEL_CLEAR = 0             # nothing in or near the clearance envelope within free_distance
uint8 LEVEL_WARNING = 1           # confirmed object close to the envelope, outside it
uint8 LEVEL_DANGER = 2            # confirmed object inside the envelope: brake

uint8 level
bool obstacle_detected            # level == LEVEL_DANGER

float32 nearest_distance          # [m] along the track to the nearest DANGER object, NaN if none
float32 nearest_lateral           # [m] its offset from the track axis (left positive), NaN if none
float32 time_to_collision         # [s] for the nearest DANGER object, NaN if not approaching
float32 confidence                # [0..1] of the nearest DANGER object, 0 if none
float32 free_distance             # [m] length of track ahead that is observed and free of obstacles

Obstacle[] obstacles              # all confirmed objects (DANGER and WARNING), nearest first

# diagnostics
float32 processing_time_ms        # callback start -> result
float32 transport_latency_ms      # publisher source timestamp -> callback start (NaN if unknown)
uint32 num_points                 # valid points in the frame
float32 sensor_height             # [m] estimated lidar height above the track bed
float32 sensor_pitch_deg
float32 sensor_roll_deg
float32 track_curvature           # [1/m] curvature of the track near the train (left positive)
bool track_axis_locked            # near-field track axis found from drainage channel / rails
float32 train_speed               # [m/s] measured from the lidar alone (structures sliding past), NaN if
                                  #   the frame carries no signature to match
