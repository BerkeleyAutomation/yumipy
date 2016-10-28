from yumipy import YuMiRobot, YuMiCommException

import IPython

if __name__ == '__main__':

	y = YuMiRobot()

	print 'Initial ping'
	pong = y.right.ping()
	print 'Pong: {0}'.format(pong)

	_ = raw_input("Please shut off YuMi's server. Hit [ENTER] When done.")

	try:
		pong = y.right.ping()
	except YuMiCommException, e:
		print "Got exception {0}".format(e)
		_ = raw_input("Please turn on YuMi's server. Hit [ENTER] when done.")

	print 'Resting'
	y.reset()
	print 'Test ping'
	pong = y.right.ping()
	print 'Pong: {0}'.format(pong)

	IPython.embed()
	exit(0)