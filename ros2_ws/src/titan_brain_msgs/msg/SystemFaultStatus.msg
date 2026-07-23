# Explicit fault state consumed by TB-EVAL-007B priority arbitration.
uint8 FAULT_OK=0
uint8 FAULT_E_STOP_ACTIVE=1
uint8 FAULT_HARDWARE_FAULT=2
uint8 FAULT_LATCHED_SAFETY_FAULT=3

# Source timestamp used for strict freshness and future-time validation.
std_msgs/Header header
uint8 fault_state
