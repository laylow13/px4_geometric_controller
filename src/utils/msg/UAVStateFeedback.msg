std_msgs/Header header

#uint8 FRAME_WORLD_NED=0
#uint8 FRAME_WORLD_ENU=1
uint8 world_frame

#uint8 FRAME_BODY_FRD=0
#uint8 FRAME_BODY_FLU=1
uint8 body_frame

geometry_msgs/Vector3 pos         #position in world frame
geometry_msgs/Vector3 vel     #velocity
geometry_msgs/Vector3 acc    #accleration
geometry_msgs/Vector3 jerk    #jerk
geometry_msgs/Vector3 snap    #snap
geometry_msgs/Quaternion q      #rotation of the body frame relative to the world frame
geometry_msgs/Vector3 ang_vel         #angular velocity expressed in body frame
geometry_msgs/Vector3 ang_acc     #angular velocity expressed in body frame