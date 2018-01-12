"""
Test for YuMi delta rotation
Author: Jeff Mahler
"""
from autolab_core import RigidTransform
from autolab_core import transformations as tf
import IPython
import numpy as np
import time
from yumipy import YuMiRobot
from yumipy import YuMiConstants as YMC

if __name__ == '__main__':
    robot = YuMiRobot()
    robot.set_z('fine')

    arm = robot.left

    """
    arm.goto_pose_delta([0,0,0.01])
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,0,90])
    IPython.embed()    
    arm.goto_pose_delta([0,0,0],
                        rotation=[30,0,0])
    IPython.embed()
    arm.goto_pose_delta([0,0,0],
                        rotation=[-30,0,0])
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,0,-90])
    arm.goto_pose_delta([0,0,-0.01])

    robot.stop()
    exit(0)
    """
    """
    R_cur_world = np.array([[0,1,0],
                            [1,0,0],
                            [0,0,-1]])
    T_cur_world = RigidTransform(rotation=R_cur_world)
    """
    T_cur_world = arm.get_pose()
    rx = np.pi / 4
    ry = np.pi / 8
    rz = -np.pi / 2
    R_x = RigidTransform.x_axis_rotation(rx)
    R_y = RigidTransform.y_axis_rotation(ry)
    R_z = RigidTransform.z_axis_rotation(rz)
    R = R_z.dot(R_y).dot(R_z)
    #R = tf.euler_matrix(rx, ry, rz)[:3,:3]
    R_des_world = T_cur_world.rotation.dot(R)
    t = 0.01 * 2 * (np.random.rand(3) - 0.5)
    t_des_world = T_cur_world.translation + t
    T_des_world = RigidTransform(rotation=R_des_world,
                                 translation=t_des_world,
                                 from_frame=T_cur_world.from_frame,
                                 to_frame=T_cur_world.to_frame)

    T_des_cur = T_cur_world.inverse() * T_des_world
    T_cur_des = T_des_cur.inverse()
    delta_t = T_cur_world.rotation.dot(T_des_cur.translation)
    #delta_R = T_cur_world.rotation.dot(T_des_cur.rotation)
    #T = RigidTransform(rotation=delta_R)
    #print T.euler_angles
    print T_des_cur.euler
    print T_cur_des.euler

    rx, ry, rz = T_des_cur.euler
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,0,rz])
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,ry,0])
    arm.goto_pose_delta([0,0,0],
                        rotation=[rx,0,0])
    IPython.embed()
    rx, ry, rz = T_cur_des.euler
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,0,rz])
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,ry,0])
    arm.goto_pose_delta([0,0,0],
                        rotation=[rx,0,0])
    """
    arm.goto_pose_delta([0,0,0],
                        rotation=[-rx,0,0])
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,-ry,0])
    arm.goto_pose_delta([0,0,0],
                        rotation=[0,0,-rz])
    """
    IPython.embed()

    robot.stop()
