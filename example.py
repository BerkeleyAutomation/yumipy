from yumipy.yumi_robot import YuMiRobot
from yumipy.yumi_state import YuMiState
from autolab_core import RigidTransform
import numpy as np
from yumiplanning.yumi_kinematics import YuMiKinematics as YK
y=YuMiRobot()
l_nice_state=YuMiState(np.rad2deg(YK.urdf_order_2_yumi(YK.L_NICE_STATE)))
r_nice_state=YuMiState(np.rad2deg(YK.urdf_order_2_yumi(YK.R_NICE_STATE)))
y.set_v(300,200)#set speed (lienar mm/s, rotational deg/s)
input("Enter to move arms to home")
y.left.goto_state(l_nice_state)
y.right.goto_state(r_nice_state)
input("Enter to close and open the gripper")
y.left.close_gripper()
y.left.open_gripper()
input('Enter to run waypoints')

print("Computing waypoints")
#poses that we want the arms to follow
L_TWIST_POSE = RigidTransform(translation=(.35,.1,.18),rotation=[0.2439,.95426,.16741,.04325],from_frame='gripper',to_frame='world')
R_TWIST_POSE = RigidTransform(translation=(.35,-.12,.18),rotation=[0.3776,-.92516,.02571,-.02827],from_frame='gripper',to_frame='world')
L_OUT_POSE=RigidTransform(translation=(.35,.4,.3),rotation=[0,1,0,0],from_frame='gripper_l_base',to_frame='base_link')
R_OUT_POSE=RigidTransform(translation=(.35,-.3,.3),rotation=[0,1,0,0],from_frame='gripper',to_frame='world')
lpts=[YK.L_NICE_POSE,L_TWIST_POSE,L_OUT_POSE,YK.L_NICE_POSE]
rpts=[YK.R_NICE_POSE,R_TWIST_POSE,R_OUT_POSE,YK.R_NICE_POSE]

#compute the actual path (THESE ARE IN URDF ORDER (see urdf_order_2_yumi for details))
yk = YK()
lpath,rpath=yk.interpolate_cartesian_waypoints(lpts,rpts,YK.L_NICE_STATE,YK.R_NICE_STATE)
#convert to yumi joint order and degrees
l_waypoints=[np.rad2deg(YK.urdf_order_2_yumi(q)) for q in lpath]
r_waypoints=[np.rad2deg(YK.urdf_order_2_yumi(q)) for q in rpath]

print("Sending poses to arm")
#send the waypoints to the yumi and execute them
y.left.joint_buffer_clear()
y.right.joint_buffer_clear()
for i in range(len(r_waypoints)):
	y.left.joint_buffer_add(YuMiState(l_waypoints[i]))
	y.right.joint_buffer_add(YuMiState(r_waypoints[i]))
y.left.joint_buffer_execute(wait_for_res=False)
y.right.joint_buffer_execute()