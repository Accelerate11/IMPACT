uint8 OK=0
uint8 WARN=1
uint8 FAIL=2

std_msgs/Header header
string module_id
uint64 seq
uint8 state
float64 age_ms
float64 latency_ms
float64 quality
string error_code
string detail

