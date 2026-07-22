# Audit status emitted for every authoritative velocity arbitration.
uint8 MODE_PASS_THROUGH=0
uint8 MODE_CLAMPED=1
uint8 MODE_FORCED_ZERO=2

std_msgs/Header header
uint8 mode
string reason
string policy_version
string correlation_id
bool is_safe
uint64 command_sequence_id
uint64 safety_intent_sequence_id
float64 max_abs_linear_x
float64 max_abs_linear_y
float64 max_abs_angular_z
float64 warning_max_abs_linear_x
float64 warning_max_abs_linear_y
float64 warning_max_abs_angular_z
geometry_msgs/Twist commanded_twist
