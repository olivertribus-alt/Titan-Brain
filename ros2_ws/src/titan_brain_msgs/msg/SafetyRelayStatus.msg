# Commanded and observed state of the external safety relay.
std_msgs/Header header

uint8 COMMANDED_CLOSED=1
uint8 COMMANDED_OPEN=2
uint8 commanded_state
uint8 FEEDBACK_UNKNOWN=0
uint8 FEEDBACK_CLOSED=1
uint8 FEEDBACK_OPEN=2
uint8 feedback_state
bool is_latched
