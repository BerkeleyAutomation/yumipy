"""
Test code for the YuMi suction motion planner
Author: Jeff Mahler
"""
import IPython
import numpy as np
import time

from yumipy import YuMiRobot
from autolab_core import RigidTransform

from visualization import Visualizer3D as vis

if __name__ == '__main__':
    random_rotation = False

    r = YuMiRobot(use_suction=True)
    if random_rotation:
        R = RigidTransform.random_rotation()
        while R[2,2] > -0.1:
            R = RigidTransform.random_rotation()
    else:
        v = 2 * (np.random.rand(3) - 0.5)
        if v[2] > 0:
            v = -v
        v = v / np.linalg.norm(v)
        x = np.array([-v[1], v[0], 0])
        x = x / np.linalg.norm(x)
        z = v
        y = np.cross(z, x)
        R = np.c_[x, y, z]
    t = np.array([0.4, -0.2, 0.2])
    T_suction_world = RigidTransform(rotation=R,
                                     translation=t,
                                     from_frame='tool',
                                     to_frame='world')

    r.right.goto_pose(T_suction_world)
    r.right.suction_on()
    time.sleep(1)
    r.right.suction_off()
    r.stop()
