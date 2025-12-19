# RobotStatus.msg
# Status message sent by robots to brain

string robot_name
float32 battery_level
geometry_msgs/Point current_location
bool is_available
string current_task

