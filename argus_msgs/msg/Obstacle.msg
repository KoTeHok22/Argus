

std_msgs/Header header

uint32 track_id
uint32 persistence_frames

geometry_msgs/Point   position
geometry_msgs/Vector3 extent
float32 volume_m3

float32 range_m
float32 centroid_range_m
float32 ttc_s

float32 confidence
uint8   votes_free_space
uint8   votes_no_return
uint8   votes_geometry
uint32  point_count

uint8 severity
uint8 SEVERITY_INFO     = 0
uint8 SEVERITY_WARNING  = 1
uint8 SEVERITY_CRITICAL = 2

string reason
