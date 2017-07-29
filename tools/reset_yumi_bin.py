"""
Helper script to move YuMi back to home pose
Author: Jeff Mahler, Jacky Liang
"""
import argparse
from yumipy import YuMiRobot
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
    y.left.goto_state(YMC.L_KINEMATIC_AVOIDANCE_STATE)

    T_cur = y.left.get_pose()
    delta_t = YMC.L_BIN_PREGRASP_POSE.translation - T_cur.translation
    #delta_t = [0, 0, 0.1]
    #y.left.goto_pose_delta(delta_t)

    #y.left.open_gripper()
    y.stop()
