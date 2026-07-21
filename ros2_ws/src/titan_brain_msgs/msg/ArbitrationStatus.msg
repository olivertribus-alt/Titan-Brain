# Audit status emitted for every authoritative velocity arbitration.
uint8 MODE_PASS_THROUGH=0
uint8 MODE_CLAMPED=1
uint8 MODE_FORCED_ZERO=2

std_msgs/Header header
uint8 mode
string reason
geometry_msgs/Twist commanded_twist
