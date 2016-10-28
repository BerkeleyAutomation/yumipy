from yumipy import YuMiRobot

if __name__ == '__main__':
	y = YuMiRobot()
	y.calibrate_grippers()
	y.open_grippers()
	y.stop()