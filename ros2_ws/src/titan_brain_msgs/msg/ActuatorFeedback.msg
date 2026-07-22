# Measured actuator response consumed by TB-ACT-001C.
uint8 STATE_STOPPED=0
uint8 STATE_MOVING=1
uint8 STATE_INVALID_DATA=2
uint8 STATE_STALE_DATA=3

std_msgs/Header header
string schema_version
string correlation_id
uint64 sequence_id
float64 measured_linear_x
float64 measured_linear_y
float64 measured_angular_z
uint8 state
bool is_stopped
bool is_fresh
bool is_valid
