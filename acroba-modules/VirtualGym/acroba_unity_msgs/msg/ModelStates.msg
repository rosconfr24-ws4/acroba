# broadcast all model states in world frame
std_msgs/Header header        # header
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Point[] scale