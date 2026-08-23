std_msgs/Header header

# Local-map surfels available to P10. Arrays are index-aligned.
geometry_msgs/Point[] positions
geometry_msgs/Vector3[] normals
float64[] static_confidence
float64[] geometry_quality
float64[] last_seen_s

bool valid
string source
bool ground_truth_used
