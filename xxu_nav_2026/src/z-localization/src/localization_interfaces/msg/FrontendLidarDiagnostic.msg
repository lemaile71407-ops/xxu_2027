# Optional tracking diagnostic, one per processed observation attempt after initialization.
# Identity and time match FrontendFrame. No data is substituted for failed initialization.
uint64 instance_id
uint64 epoch
uint64 sequence
builtin_interfaces/Time begin

# Same-time IMU prediction and committed estimate. Pose/covariance: world_frame,
# physical [x,y,z,theta_x,theta_y,theta_z], m/rad; twist: body_frame as in /odometry.
nav_msgs/Odometry prediction
nav_msgs/Odometry corrected

# Last actual matcher/filter linearization, expressed in prediction.header.frame_id.
geometry_msgs/Pose linearization_pose
uint32 sampled_points
uint32 matches
uint32 linearizations
bool accepted

# Pure LiDAR measurement information, excluding the prior. Both 6x6 arrays are ROW-MAJOR.
# Solver tangent [rho_p,theta] obeys dp = rho_p - skew(p_linearization)*theta.
float64[36] solver_information
# Physical tangent [dp_world,theta_world], order x,y,z,theta_x,theta_y,theta_z.
# H_physical = B^T H_solver B, B=[I,skew(p_linearization);0,I]. Mixed m/rad units.
# This is NOT the inverse posterior covariance; anchor is linearization_pose.
float64[36] observation_information

# Direct differences: p_corrected-p_prediction and Log(R_corrected*R_prediction^T).
# World axes, meters and radians; rotation is a rotation vector, not Euler angles.
geometry_msgs/Vector3 position_correction_world
geometry_msgs/Vector3 rotation_correction_world

# Full 15x15 ROW-MAJOR internal SD covariance before/after fusion, respectively
# anchored at prediction/corrected. Order [rho_p,theta,rho_v,beta_g,beta_a].
# Use each Odometry.pose.covariance above for physical 6x6 world pose covariance.
float64[225] predicted_covariance
float64[225] corrected_covariance
