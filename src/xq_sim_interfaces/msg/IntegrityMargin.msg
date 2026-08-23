std_msgs/Header header

int64 trajectory_id
uint32 trajectory_sample_count
uint32 critical_sample_index

geometry_msgs/Point critical_sample_point
geometry_msgs/Point nearest_obstacle_point
float64[3] obstacle_direction_map

float64 alert_limit
float64 protection_level
float64 minimum_margin
float64 margin_reserve
float64 k_alpha

float64[] sample_protection_levels
float64[] sample_margins

bool accepted
bool valid
bool hard_constraint
string decision_reason
string calibration_sha256
