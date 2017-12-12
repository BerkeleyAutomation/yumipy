"""
Test for YuMi delta rotation
Author: Jeff Mahler
"""
from autolab_core import RigidTransform
import numpy as np
import IPython
from yumipy import YuMiRobot
from yumipy import YuMiConstants as YMC

if __name__ == '__main__':
    y = YuMiRobot()
    y.set_z('fine')

    y.right.goto_state(YMC.R_KINEMATIC_AVOIDANCE_STATE)
    T_cur = y.right.get_pose()
    delta_t = YMC.R_BIN_PREGRASP_POSE.translation - T_cur.translation
    y.right.goto_pose_delta(delta_t)

    T_cur = y.right.get_pose()
    delta_R = RigidTransform.x_axis_rotation(np.pi/16)
    delta_T = RigidTransform(rotation=delta_R, from_frame='tool',
                             to_frame='tool')
    T_des = T_cur * delta_T
    
    #y.right.goto_pose(T_des)
    IPython.embed()

    y.stop()
