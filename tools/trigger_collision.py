import numpy as np
import time

from yumipy import YuMiRobot, YuMiState
from autolab_core import RigidTransform

if __name__ == '__main__':
	num_attempts = 3

	state = YuMiState([51.16, -99.4, 21.57, -107.19, 84.11, 94.61, -36.00])
	robot = YuMiRobot(debug=False)
	robot.set_v(50)

	for i in range(num_attempts):
		print 'Trying collision', i
		robot.right.goto_state(state)
		robot.right.goto_pose_delta([0, 0.236, 0])
		robot.right.goto_pose_delta([0.053, 0, 0])
		robot.right.goto_pose_delta([0, 0, -0.145])
		time.sleep(3)
