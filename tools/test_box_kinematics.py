"""
Script to test YuMi movements within a bin
Author: Jeff Mahler
"""
import IPython
import numpy as np
import os
import sys

from yumipy import YuMiRobot
from yumipy import YuMiConstants as YMC

if __name__ == '__main__':
    robot = YuMiRobot()
    #robot.left.goto_pose(YMC.L_PREGRASP_POSE)
    robot.left.goto_state(YMC.L_KINEMATIC_AVOIDANCE_STATE)
    #robot.left.goto_pose(YMC.L_KINEMATIC_AVOIDANCE_POSE)

    robot.left.goto_pose_delta([0,0,0], rotation=[0,0,90])

    T_orig_gripper_world = robot.left.get_pose()

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_LOWER_LEFT.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_UPPER_LEFT.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_UPPER_RIGHT.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_LOWER_RIGHT.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)    

    T_gripper_world = robot.left.get_pose()
    delta_t = T_orig_gripper_world.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)    

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_PREGRASP_POSE.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)    

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_DROP_POSE.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)    

    T_gripper_world = robot.left.get_pose()
    delta_t = YMC.L_BIN_PREGRASP_POSE.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)    

    T_gripper_world = robot.left.get_pose()
    delta_t = T_orig_gripper_world.translation - T_gripper_world.translation
    robot.left.goto_pose_delta(delta_t)    

    robot.stop()
    exit(0)

    delta = 0.05
    robot.left.goto_pose_delta([-0.1, 0.0, 0.0])
    robot.left.goto_pose_delta([0.0, 0.0, 0.0], rotation=[0.0, 0.0, 90.0])
    robot.left.goto_pose_delta([delta, 0.0, 0.0])
    robot.left.goto_pose_delta([0.0, delta, 0.0])
    robot.left.goto_pose_delta([-delta, 0.0, 0.0])
    robot.left.goto_pose_delta([0.0, -delta, 0.0])

    robot.left.goto_pose_delta([delta, 0.0, -0.12])
    robot.left.goto_pose_delta([delta, 0.0, 0.0])
    robot.left.goto_pose_delta([0.0, delta, 0.0])
    robot.left.goto_pose_delta([-delta, 0.0, 0.0])
    robot.left.goto_pose_delta([0.0, -delta, 0.0])
    robot.stop()
