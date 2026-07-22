# Authoritative evaluator output for the actuator control plane.
uint8 STATE_NORMAL=0
uint8 STATE_WARNING=1
uint8 STATE_E_STOP=2
uint8 STATE_RECOVERY_HOLDING=3

uint8 state
builtin_interfaces/Time timestamp
string correlation_id
uint64 sequence_id
