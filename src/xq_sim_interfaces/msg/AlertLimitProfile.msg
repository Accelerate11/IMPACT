std_msgs/Header header

int64 trajectory_id
uint32 trajectory_sample_count
uint32 obstacle_point_count

# Full remaining-trajectory profile for P9.  All arrays have
# trajectory_sample_count entries; directions are flattened XYZ triples.
geometry_msgs/Point[] trajectory_sample_points
geometry_msgs/Point[] sample_nearest_obstacle_points
float64[] sample_obstacle_directions_map
float64[] sample_geometric_clearances
float64[] sample_alert_limits

bool valid
bool static_obstacles_only
string obstacle_source
