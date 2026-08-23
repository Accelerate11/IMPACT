std_msgs/Header header

# FAST-LIO ESKF translational covariance and conservative P_int, row-major.
float64[9] lio_position_covariance
float64[9] integrity_covariance
float64[9] information_matrix
float64[3] geometry_eigenvalues
float64[3] weak_direction_map

float64 lambda_min
float64 condition_number
float64[3] protection_level_axes
float64 weak_direction_protection_level

# Parameterized inflation terms in kappa = 1 + a_D D + a_N NIS + a_T T_j + a_R R_d.
float64 kappa
float64 k_alpha
float64 eta
float64 epsilon
float64 degeneracy_term
float64 nnis_term
float64 timing_jitter_term
float64 residual_dynamics_term
uint32 effective_points

