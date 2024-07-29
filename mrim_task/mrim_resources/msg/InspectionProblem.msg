string name
string comment

uint32 number_of_robots
uint32 number_of_inspection_points
uint32 number_of_obstacle_points

uint32[]                         robot_ids   # IDs of the robots
mrs_msgs/Reference[]             start_poses # [x y z heading] start poses of the robots
mrim_resources/InspectionPoint[] inspection_points
geometry_msgs/Point[]            obstacle_points
geometry_msgs/Point[]            safety_area
float32                          min_height
float32                          max_height

string model_name
string mesh_path
