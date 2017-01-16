'''
Util functions for converting
'''
import logging
import numpy as np

from yumi_state import YuMiState
from yumi_constants import YuMiConstants as YMC
from core import RigidTransform

def message_to_pose(message, from_frame):
    tokens = message.split()
    try:
        if len(tokens) != 7:
            raise Exception("Invalid format for pose! Got:\n{0}".format(message))
        pose_vals = [float(token) for token in tokens]
        q = pose_vals[3:]
        t = pose_vals[:3]
        R = RigidTransform.rotation_from_quaternion(q)
        pose = RigidTransform(R, t, from_frame=from_frame)
        pose.position = pose.position * MM_TO_METERS

        return pose

    except Exception, e:
        logging.error(e)

def message_to_state(message):
    tokens = message.split()

    try:
        if len(tokens) != YuMiState.NUM_JOINTS:
            raise Exception("Invalid format for states! Got: \n{0}".format(message))
        state_vals = [float(token) for token in tokens]
        state = YuMiState(state_vals)

        return state

    except Exception, e:
        logging.error(e)

def message_to_torques(message):
    tokens = message.split()
    torque_vals = np.array([float(token) for token in tokens])

    return torque_vals

def construct_req(code_name, body=''):
    req = '{0:d} {1}#'.format(YMC.CMD_CODES[code_name], body)
    return req

def iter_to_str(template, iterable):
    result = ''
    for val in iterable:
        result += template.format(val).rstrip('0').rstrip('.') + ' '
    return result

def get_pose_body(pose):
    if not isinstance(pose, RigidTransform):
        raise ValueError('Can only parse RigidTransform objects')
    pose = pose.copy()
    pose.position = pose.position * YMC.METERS_TO_MM
    body = '{0}{1}'.format(iter_to_str('{:.1f}', pose.position.tolist()),
                                        iter_to_str('{:.5f}', pose.quaternion.tolist()))
    return body
