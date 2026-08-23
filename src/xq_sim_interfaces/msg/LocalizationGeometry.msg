std_msgs/Header header

# Weighted translational point-to-plane information matrix, row-major.
float64[9] information_matrix
float64[3] eigenvalues
float64[3] weak_direction_map
float64 lambda_min
float64 condition_number
float64 residual_rms
float64 weight_sum
uint32 effective_points

