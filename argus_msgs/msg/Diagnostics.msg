

std_msgs/Header header

uint32 n_points_raw
uint32 n_points_valid
uint32 n_points_rejected
uint32 reject_zero
uint32 reject_nonfinite
uint32 reject_out_of_range

uint32 rings_detected
uint32 az_steps_detected
uint8  echo_multiplier
uint32 point_step
bool   layout_ok
string layout_error

float32 profile_median_deviation_m
uint32  model_voxel_count
bool    model_ready

uint32 n_ground_points
float32 t_filter_ms
float32 t_range_image_ms
float32 t_ground_ms
float32 t_odometry_ms
float32 t_model_ms
float32 t_detect_ms
float32 t_cluster_ms
float32 t_track_ms
float32 t_total_ms
float32 fps

uint64 frames_processed
uint64 alerts_total
uint64 false_alerts_suppressed
