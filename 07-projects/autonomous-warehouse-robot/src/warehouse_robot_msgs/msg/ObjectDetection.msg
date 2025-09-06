# Object detection message
std_msgs/Header header
string object_type
float32 confidence
geometry_msgs/PoseStamped object_pose
geometry_msgs/Vector3 object_size
string[] grasp_points
float32 grasp_force
