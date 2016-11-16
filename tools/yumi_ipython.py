from yumipy import *
from core import *

from time import sleep

import logging
import IPython
import numpy as np

YMC = YuMiConstants

from multiprocess import Process, Queue

class P(Process):

    def __init__(self, y):
        Process.__init__(self)
        self.y = y

    def run(self):
        #self.y = YuMiRobot()
        p = self.y.left.get_pose()
        print p
        self.y.reset_home()
        print 'finished'

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    y = YuMiRobot()
    p = P(y)
    p.start()

    IPython.embed()
    y.stop()
    exit(0)
