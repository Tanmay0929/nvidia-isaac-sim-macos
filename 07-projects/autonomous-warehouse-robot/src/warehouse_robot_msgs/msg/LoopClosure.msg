# Loop closure detection message
std_msgs/Header header
int32 landmark_id
float32 confidence
geometry_msgs/PoseStamped robot_pose
geometry_msgs/PoseStamped landmark_pose
string detection_method  # "visual", "geometric", "hybrid"
