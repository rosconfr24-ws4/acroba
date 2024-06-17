int32[] controller_ids #index of the joint controller in the robot
acroba_unity_msgs/JointController[] joint_controllers #values of the Articulationbody

#CONTROL VELOCITIES
#if robot_verl>0 it will control the velocity of the robot joints in rads/s
#else if robot_vel==0 it won't change the value
#else if robot_vel<0 it will deactivate the velocity control giving full control to the Unity Drivers
float32 robot_vel 
float32 tool_vel
