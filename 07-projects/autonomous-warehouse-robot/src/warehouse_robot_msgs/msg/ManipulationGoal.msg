# Manipulation goal message
std_msgs/Header header
string operation  # "pick", "place", "move_to_pose"
geometry_msgs/PoseStamped target_pose
string object_type  # "package_small", "package_medium", "package_large"
float32 grasp_force
float32 approach_velocity
bool force_control
