
string name
int32 status
int32 type
geometry_msgs/Pose pose
float32 range # in meters only for spot/point
float32 angle #degrees only for spot
float32 intensity #[0,8] 
std_msgs/ColorRGBA color #A won't be used

#Status and actions to be performed over lights
int32 ENABLED=0
int32 DISABLED=1
int32 ADD=2
int32 REMOVE=3
int32 CHANGED=4
#Type of lights
int32 DIRECTIONAL=0
int32 SPOT=1
int32 POINT=2