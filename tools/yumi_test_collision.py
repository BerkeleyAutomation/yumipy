from yumipy import *
from yumipy import *

from time import sleep

import logging
import IPython
import numpy as np

YMC = YuMiConstants

if __name__ == '__main__':
    num_tries = 1
    logging.getLogger().setLevel(logging.INFO)

    y = YuMiRobot(include_right=False)
    y.set_v(100)

    for i in range(num_tries):
        p = y.left.goto_pose_delta([0,0,0.1])
        p = y.left.goto_pose_delta([0,0,-0.15])
    
    y.stop()
    exit(0)