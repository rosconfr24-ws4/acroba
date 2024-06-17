
#Articulation body related values Negative values means that they shoould
# remain unchanged
#F=max(force_limit,stiffness*(current_pose-target_pose)-damping*(currentVel-targetVel))
float32 stiffness
float32 damping
float32 force_limit
#Joint related values
float32 body_angular_damping
float32 body_linear_damping
float32 joint_friction