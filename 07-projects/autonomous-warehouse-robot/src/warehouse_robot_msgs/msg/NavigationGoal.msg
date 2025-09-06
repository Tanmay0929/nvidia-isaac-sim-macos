# Navigation goal message for warehouse robot
std_msgs/Header header
geometry_msgs/PoseStamped target_pose
string operation  # "navigate", "explore", "patrol"
float32 max_velocity
float32 goal_tolerance
bool dynamic_replanning
