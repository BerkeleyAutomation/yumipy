"""
Helper script to move YuMi back to home pose
Author: Jeff Mahler
"""
from yumipy import YuMiRobot, YuMiState
from yumipy import YuMiConstants as YMC

if __name__ == '__main__':
    y = YuMiRobot()
    y.set_z('fine')
    y.set_v(200)

    y.left.close_gripper()
    y.right.close_gripper()

    state = YuMiState([69.42,-75.37,20.21,-110.77,91.29,-62.35,-43.74])
    #y.left.goto_pose(YMC.L_HOME_POSE)
    y.right.goto_state(state)
    #y.stop()
    #exit(0)

    for i, tup in enumerate(YMC.BOX_CLOSE_SEQUENCE):
        print i
        arm, box_state = tup
        if arm == 'L':
            if isinstance(box_state, YuMiState):
                y.left.goto_state(box_state)
            elif isinstance(box_state, list):
                y.left.goto_pose_delta(box_state)
            else:
                y.left.goto_pose(box_state)
        else:
            if isinstance(box_state, YuMiState):
                y.right.goto_state(box_state)
            elif isinstance(box_state, list):
                y.right.goto_pose_delta(box_state)
            else:
                y.right.goto_pose(box_state)

    y.stop()
