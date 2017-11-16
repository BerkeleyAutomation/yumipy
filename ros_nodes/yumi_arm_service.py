#!/usr/bin/env python
"""
ROS node with service that allows for controlling the YuMi remotely
"""
from sensor_msgs.msg import JointState

import argparse
import rospy
import numpy as np

try:
    from yumipy.yumi_arm import *
    from yumipy.srv import *
except ImportError:
    raise RuntimeError(
        "yumi_ros_service unavailable outside of catkin package")

if __name__ == '__main__':
    # Initialize server. Name is generic as it will be overwritten by launch anyways
    rospy.init_node('arm_server')

    # Arguments:
    # name:    string,           name of arm to initialize a server for
    # verbose: bool, optional    if True, logs a line for every function call handled.
    name = rospy.get_param('~name')
    verbose = rospy.get_param('~display_output', True)

    # Get local YuMiArm and its method dict
    arm = YuMiArmFactory.YuMiArm('local', name)
    yumi_methods = YuMiArm.__dict__

    # Define how requests are handled (Call the corresponding method in the local class)
    def handle_request(req):
        func = pickle.loads(req.func)
        args = pickle.loads(req.args)
        kwargs = pickle.loads(req.kwargs)

        if func == '__getattribute__':
            if verbose:
                rospy.loginfo(
                    "Handling request to get attribute {0} for {1} arm".format(
                        args, name))
            return ROSYumiArmResponse(pickle.dumps(getattr(arm, args)))
        else:
            if verbose:
                rospy.loginfo(
                    "Handling request to call method {0} for {1} arm".format(
                        func, name))
            return ROSYumiArmResponse(
                pickle.dumps(yumi_methods[func](arm, *args, **kwargs)))

    s = rospy.Service('{0}_arm'.format(name), ROSYumiArm, handle_request)
    rospy.loginfo("{0} arm is ready".format(name))

    joint_state_pub = rospy.Publisher(
        '/joint_states', JointState, queue_size=10)

    rate = rospy.Rate(20)

    # Keep process alive
    while not rospy.is_shutdown():
        joints = arm.get_state()
        if joints:
            joint_names = [
                'yumi_joint_{}_{}'.format(i, name[0]) for i in range(1, 8)
            ]

            ros_now = rospy.Time.now()
            joint_state_message = JointState()
            joint_state_message.header.stamp = ros_now
            joint_state_message.name = joint_names
            joint_state_message.position = np.radians(joints.joints)
            joint_state_pub.publish(joint_state_message)
        rate.sleep()
