'''
Constants for YuMi interface and control
Author: Jacky Liang
'''
import logging
from .yumi_state import YuMiState
from autolab_core import RigidTransform

class YuMiConstants:

    IP = '192.168.125.1'

    PORTS = {
        "left" : {
            "server":5000,
            "states":5010,
            "poses":5012,
            "torques":5014
        },
        "right" : {
            "server":5001,
            "states":5011,
            "poses":5013,
            "torques":5015
        },
    }
    
    BUFSIZE = 4096
    MOTION_TIMEOUT = 10
    COMM_TIMEOUT = 1
    PROCESS_TIMEOUT = 10
    PROCESS_SLEEP_TIME = 0.005

    MAX_GRIPPER_WIDTH = 0.02
    MAX_GRIPPER_FORCE = 20
    
    DEBUG = False
    LOGGING_LEVEL = logging.WARNING
    
    # reset mechanism
    RESET_RIGHT_COMM = '/dev/ttyACM0'
    RESET_BAUDRATE = 115200
    
    CMD_CODES = {
        'ping': 0,
        'goto_pose_linear':1,
        'goto_joints': 2,
        'get_pose': 3,
        'get_joints':4,
        'goto_pose':5,
        'set_tool':6,
        'set_speed':8,
        'set_zone':9,
        
        'goto_pose_sync':11,
        'goto_joints_sync':12,
        'goto_pose_delta':13,
        
        'close_gripper': 20,
        'open_gripper': 21,
        'calibrate_gripper': 22,
        'set_gripper_max_speed': 23,
        'set_gripper_force': 24,
        'move_gripper': 25,
        'get_gripper_width': 26,
        
        'set_circ_point':35,
        'move_by_circ_point':36,
        'buffer_add': 30,
        'buffer_clear': 31,
        'buffer_size': 32,
        'buffer_move': 33,
        'joint_buffer_add':101,
        'joint_buffer_clear':102,
        'joint_buffer_move':103,
        'joint_buffer_size':104,
        
        'is_pose_reachable': 40,
        'is_joints_reachable': 41,

        'close_connection': 99,
    }
    
    RES_CODES = {
        'failure': 0,
        'success': 1
    }

    SUB_CODES = {
        'pose': 0,
        'state': 1
    }
    #by default the TCP is inside the wrist frame (called gripper_l_base and gripper_r_base in the urdf)
    TCP_DEFAULT = RigidTransform(from_frame="tcp",to_frame="gripper_base")