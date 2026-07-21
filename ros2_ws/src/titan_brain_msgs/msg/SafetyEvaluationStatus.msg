# Audit-friendly status emitted by the Titan Brain ROS transport layer.
# header.stamp is the evaluation time and header.frame_id is the target frame.
std_msgs/Header header
string schema_version
string adapter_status
string watchdog_status
bool watchdog_healthy
bool observation_accepted
string decision_id
string action
string rule
bool is_incident
string detail
