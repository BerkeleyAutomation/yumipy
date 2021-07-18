from yumipy.yumi_robot import YuMiRobot
from yumipy.yumi_state import YuMiState
from autolab_core import RigidTransform
import numpy as np
from yumiplanning.yumi_kinematics import YuMiKinematics as YK
y=YuMiRobot()
l_nice_state=YuMiState(np.rad2deg(YK.urdf_order_2_yumi(YK.L_NICE_STATE)))
r_nice_state=YuMiState(np.rad2deg(YK.urdf_order_2_yumi(YK.R_NICE_STATE)))
grip_down_r=np.diag([1,-1,-1])
y.set_v(300,360)#set speed (linear mm/s, rotational deg/s)
c=input("Move arms to home?(y/n)")
if "y" in c:
	y.left.goto_state(l_nice_state)
	y.right.goto_state(r_nice_state)

print("Type in coords for the left arm to move to!")
yk = YK()
while True:
	xpos = float(input("Enter x coord:"))
	ypos = float(input("Enter y coord:"))
	zpos = float(input("Enter z coord:"))
	curpose = y.left.get_pose()
	target=RigidTransform(translation=[xpos,ypos,zpos],rotation=grip_down_r)
	lpts=[curpose,target]

	#compute the actual path (THESE ARE IN URDF ORDER (see urdf_order_2_yumi for details))
	cur_state = y.left.get_state().urdf_format
	lpath,_=yk.interpolate_cartesian_waypoints(l_waypoints=lpts,l_qinit=cur_state,N=20)
	#convert to yumi joint order and degrees
	l_waypoints=[np.rad2deg(YK.urdf_order_2_yumi(q)) for q in lpath]
	input("Press enter to send poses to left arm")
	#send the waypoints to the yumi and execute them
	y.left.joint_buffer_clear()
	for i in range(len(l_waypoints)):
		y.left.joint_buffer_add(YuMiState(l_waypoints[i]))
	y.left.joint_buffer_execute(wait_for_res=True)
