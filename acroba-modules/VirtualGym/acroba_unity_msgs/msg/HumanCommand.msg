string human_name #Human identifier
uint8 command #instruction to be sent
uint8 CREATE=0 #Create human in scene
uint8 DESTROY=1 #destroy human in the scene
uint8 MOVE=2 #move human in scene
geometry_msgs/Pose2D pose #pose to use in create and move