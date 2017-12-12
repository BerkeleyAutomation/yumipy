"""
Helper script to move YuMi back to home pose
Author: Jeff Mahler, Jacky Liang
"""
import argparse
import IPython
from yumipy import YuMiRobot, YuMiState
from yumipy import YuMiConstants as YMC

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--left', type=str, default='True')
    parser.add_argument('-r', '--right', type=str, default='True')
    args = parser.parse_args()

    def _is_true(arg):
    	return arg.lower() in ['y','yes','true','1', 't']

    y = YuMiRobot(include_right=_is_true(args.right), include_left=_is_true(args.left))

    y.set_z('fine')
    y.right.goto_state(YuMiState([62.62,-20.31,44.91,-78.61,74.79,-90.07,-36.71]))
    #y.left.goto_pose(YMC.L_PREGRASP_POSE)
    #y.left.goto_pose(YMC.L_BIN_PREGRASP_POSE)
    print y.right.get_state()
    
    IPython.embed()
    #y.open_grippers()
    y.stop()
