std_msgs/Header header

#uint8 FRAME_WORLD_NED=0
#uint8 FRAME_WORLD_ENU=1
uint8 world_frame

#uint8 FRAME_BODY_FRD=2
#uint8 FRAME_BODY_FLU=3
uint8 body_frame

geometry_msgs/Vector3 pos            #desired position in world frame
geometry_msgs/Vector3 vel        #desired velocity
geometry_msgs/Vector3 acc       #desired acceleration
geometry_msgs/Vector3 jerk       #desired jerk
geometry_msgs/Vector3 snap       #desired snap
geometry_msgs/Vector3 heading           #desired orientation of body X axis
geometry_msgs/Vector3 heading_dot
geometry_msgs/Vector3 heading_2dot