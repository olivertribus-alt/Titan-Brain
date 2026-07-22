# Stop execution result and persistent hardware-assurance status.
uint8 STATE_IDLE=0
uint8 STATE_STOP_PENDING=1
uint8 STATE_STOP_ACKNOWLEDGED=2
uint8 STATE_HARDWARE_FAULT_LATCH=3
uint8 PRIORITY_NORMAL=0
uint8 PRIORITY_CRITICAL=255

std_msgs/Header header
string schema_version
uint8 state
string reason
string correlation_id
uint64 request_sequence_id
uint64 feedback_sequence_id
bool is_stopped
bool latched_fault
bool critical
uint8 priority
uint64 stop_elapsed_ns
uint64 evaluated_timestamp_ns
