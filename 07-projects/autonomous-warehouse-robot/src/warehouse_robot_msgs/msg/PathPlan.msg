# Path planning message
std_msgs/Header header
nav_msgs/Path global_path
nav_msgs/Path local_path
float32 path_cost
float32 planning_time
bool path_valid
string planning_algorithm  # "astar", "rrt", "dijkstra"
