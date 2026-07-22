# Auditable state emitted by the external safety-loop supervisor.
std_msgs/Header header

uint8 STATE_INITIALIZING=0
uint8 STATE_OK=1
uint8 STATE_TRIPPED=2
uint8 STATE_HARDWARE_FAULT_LATCH=3

uint8 supervisor_state
string[] active_faults
bool relay_closed_request
