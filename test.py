from yumipy.yumi_robot import YuMiRobot
from yumipy.yumi_state import YuMiState
from autolab_core import RigidTransform
import numpy as np
from yumiplanning.yumi_kinematics import YuMiKinematics as YK
y=YuMiRobot()
l_nice_state=YuMiState(np.rad2deg(YK.urdf_order_2_yumi(YK.L_NICE_STATE)))
r_nice_state=YuMiState(np.rad2deg(YK.urdf_order_2_yumi(YK.R_NICE_STATE)))
grip_down_r=np.diag([1,-1,-1])
y.set_v(100,800)#set speed (linear mm/s, rotational deg/s)
c=input("Move arms to home?(y/n)")
if "y" in c:
	y.left.goto_state(l_nice_state)
	y.right.goto_state(r_nice_state)

print("Type in coords for the left arm to move to!")
yk = YK()
L_TCP = RigidTransform(translation=[0,0,.174],from_frame=yk.l_tcp_frame,to_frame=yk.l_tip_frame)
R_TCP = RigidTransform(translation=[0,0,.14],from_frame=YK.r_tcp_frame,to_frame=YK.r_tip_frame)

yk.set_tcp(L_TCP,R_TCP)
y.set_tcp(left=L_TCP,right=R_TCP)
while True:
	xpos = float(input("Enter x coord:"))
	ypos = float(input("Enter y coord:"))
	zpos = float(input("Enter z coord:"))
	cur_state = y.right.get_state().urdf_format
	_,curpose = yk.fk(qright=cur_state)
	target=RigidTransform(translation=[xpos,ypos,zpos],rotation=grip_down_r,
            from_frame=yk.r_tcp_frame,to_frame=yk.base_frame)
	rpts=[curpose,target]

	#compute the actual path (THESE ARE IN URDF ORDER (see urdf_order_2_yumi for details))
	_,rpath=yk.interpolate_cartesian_waypoints(r_waypoints=rpts,r_qinit=cur_state,N=20)
	#convert to yumi joint order and degrees
	r_waypoints=[np.rad2deg(YK.urdf_order_2_yumi(q)) for q in rpath]
	input("Press enter to send poses to right arm")
	#send the waypoints to the yumi and execute them
	y.right.joint_buffer_clear()
	for i in range(len(r_waypoints)):
		y.right.joint_buffer_add(YuMiState(r_waypoints[i]))
	y.right.joint_buffer_execute(wait_for_res=True)
	print(yk.fk(qright=y.right.get_state().urdf_format))