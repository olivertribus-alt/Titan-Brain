# Diagnostic status for TB-EVAL-003 hysteresis and recovery transitions.
uint8 STATE_OK=0
uint8 STATE_WARNING=1
uint8 STATE_E_STOP=2
uint8 STATE_RECOVERY_HOLDING=3

std_msgs/Header header
string schema_version
uint8 state
string reason
string instantaneous_action
string effective_action
bool recovery_active
uint64 hold_elapsed_ns
uint64 recovery_hold_time_ns
bool has_release_threshold
float64 release_threshold_m
