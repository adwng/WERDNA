# std_msgs/Header header

# states represent the robot's states
# start balancing
# button 3
bool state

#robot's pose
# axes 2, 3, button 0 to enable side height
float32 height 
float32 displacement

#robot's drive axes 0, 1 , button 6 turbo mode
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular

