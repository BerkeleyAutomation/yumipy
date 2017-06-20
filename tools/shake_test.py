"""
Helper script to move YuMi back to home pose
Author: Jeff Mahler
"""
from autolab_core import RigidTransform
import numpy as np

from yumipy import YuMiRobot, YuMiState
from yumipy import YuMiConstants as YMC

if __name__ == '__main__':
    y = YuMiRobot()
    y.set_z('z50')
    y.set_v(1000)

    y.left.close_gripper()

    robot = y
    arm = y.left
    arm.goto_pose(YMC.L_PREGRASP_POSE, wait_for_res=True)
    
    # shake test
    radius = 0.2
    angle = np.pi / 64
    delta_T = RigidTransform(translation=[0,0,radius], from_frame='gripper', to_frame='gripper')
    R_shake = np.array([[1, 0, 0],
                        [0, np.cos(angle), -np.sin(angle)],
                        [0, np.sin(angle), np.cos(angle)]])
    delta_T_up = RigidTransform(rotation=R_shake, translation=[0,0,-radius], from_frame='gripper', to_frame='gripper')
    delta_T_down = RigidTransform(rotation=R_shake.T, translation=[0,0,-radius], from_frame='gripper', to_frame='gripper')
    T_shake_up = YMC.L_PREGRASP_POSE.as_frames('gripper', 'world') * delta_T_up * delta_T
    T_shake_down = YMC.L_PREGRASP_POSE.as_frames('gripper', 'world') * delta_T_down * delta_T

    for i in range(5):
        arm.goto_pose(T_shake_up, wait_for_res=False)
        arm.goto_pose(YMC.L_PREGRASP_POSE, wait_for_res=True)
        arm.goto_pose(T_shake_down, wait_for_res=False)
        arm.goto_pose(YMC.L_PREGRASP_POSE, wait_for_res=True)
    y.stop()
