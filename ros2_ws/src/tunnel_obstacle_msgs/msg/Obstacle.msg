# One confirmed object near or inside the train clearance envelope.
# Track frame: X along the track ahead of the lidar, Y to the left, Z up from the track bed.

uint32 id
uint8 level                       # ObstacleStatus.LEVEL_WARNING or ObstacleStatus.LEVEL_DANGER

float32 distance                  # [m] along the track from the lidar to the nearest point of the object
float32 lateral                   # [m] offset of the object centre from the track axis (left positive)
float32 height_min                # [m] lowest point above the track bed
float32 height_max                # [m] highest point above the track bed
float32 length                    # [m] extent along the track
float32 width                     # [m] extent across the track

uint32 num_points                 # points in the current frame
uint32 hits                       # frames in which the object was observed
float32 confidence                # [0..1]
float32 relative_speed            # [m/s] d(distance)/dt, negative when approaching

geometry_msgs/Point position      # object centre in the point cloud frame
