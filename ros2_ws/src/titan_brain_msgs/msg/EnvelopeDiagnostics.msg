# Auditable evidence emitted with every TB-EVAL-008B motion envelope.
uint8 STATE_FAIL_CLOSED=0
uint8 STATE_PROTECTIVE_STOP=1
uint8 STATE_LIMITED=2
uint8 STATE_NOMINAL=3
uint8 ZONE_NONE=0
uint8 ZONE_FORWARD=1
uint8 ZONE_LATERAL=2
uint8 ZONE_SENSOR=3
uint8 ZONE_SYSTEM_FAULT=4
uint8 ZONE_TIMING=5

std_msgs/Header header
string policy_version
string correlation_id
uint64 sequence_id
uint8 state
uint8 limiting_zone
string reason
bool scan_valid
bool fault_status_valid
float64 scan_age_sec
float64 distance_forward_m
float64 distance_lateral_m
float64 linear_stopping_distance_m
float64 angular_stopping_distance_m
float64 max_linear_velocity_mps
float64 max_angular_velocity_radps
