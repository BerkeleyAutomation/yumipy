from yumipy import YuMiRobot

if __name__ == '__main__':
	y = YuMiRobot()
	y.right.calibrate_gripper()
	y.right.open_gripper()
	y.stop()
