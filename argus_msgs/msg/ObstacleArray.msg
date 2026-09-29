

std_msgs/Header header

uint8 status
uint8 STATUS_CLEAR    = 0
uint8 STATUS_WARNING  = 1
uint8 STATUS_BLOCKED  = 2
uint8 STATUS_DEGRADED = 3

float32 nearest_range_m
uint32  obstacles_detected
Obstacle[] obstacles
Candidate[] candidates

float32 processing_ms
float32 fps

bool model_ready

string explain
