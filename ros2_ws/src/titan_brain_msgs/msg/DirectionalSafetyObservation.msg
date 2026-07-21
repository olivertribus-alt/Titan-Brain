# Directional observation consumed by the opt-in dynamic braking policy.
std_msgs/Header header
string map_id
geometry_msgs/Pose2D pose
float64 clearance_m
float64 confidence
string sensor_id
float64 forward_clearance_m
float64 reverse_clearance_m
float64 left_clearance_m
float64 right_clearance_m
geometry_msgs/Twist velocity
