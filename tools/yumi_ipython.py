from yumipy import *
from core import *

from time import sleep

import logging
import IPython
import numpy as np

YMC = YuMiConstants

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)

    y = YuMiRobot()
    y.set_v(100)
    #y.left.goto_pose(YMC.L_HOME_POSE, linear=False)
    
    p = RigidTransform(rotation=[0,0,1,0], translation=[ 0.57,  0.35, 0.1], from_frame='tool', to_frame='world')
    s = YuMiState([-96.07, -80.82, -12.55, 137.1, 100.06, 4.52, 135.0])
    #y.left.goto_pose(p)
    #y.left.goto_state(s)
    #s = y.left.get_state()
    #print s
    p = y.left.goto_pose_delta([0,0,0.1])
    p = y.left.get_pose()
    p.translation[2] = 0.0
    p = y.left.goto_pose_delta([0,0,-0.15])
    #y.left.goto_pose(p)
    
    IPython.embed()
    
    while True:
        try:
            y.left.open_gripper()
            _ = raw_input("Hit [ENTER] to close gripper")
            y.left.close_gripper()
            sleep(5)
        except Exception, e:
            logging.error(e)
            y.stop()
            _ = raw_input("Hit [ENTER] when YuMi is Operational")
            y = YuMiRobot()

    IPython.embed()
    y.stop()
    exit(0)