#!/usr/bin/env python
"""
ROS node with service that allows for controlling the YuMi remotely
"""

import logging
import argparse
import rospy
try:
    from yumipy.yumi_arm import *
    from yumipy.srv import *
except ImportError:
    raise RuntimeError("yumi_ros_service unavailable outside of catkin package")

if __name__ == '__main__':
    # Initialize and run argument parser 
    parser = argparse.ArgumentParser(description="Initialize a ROS yumi arm server")
    parser.add_argument('name', type = str,
                        help="The name of the yumi arm to create a service for. Must be in {'left', 'right'}")
    parser.add_argument('--service_name', nargs = 1, type=str,
                        help="The name of the node and service that will be created. Defaults to {name}_arm")
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                        help="Show a message for every method call")
    args = parser.parse_args()
    
    name = args.name
    verbose = args.verbose
    
    # Get local YuMiArm and its method dict
    arm = YuMiArmFactory.YuMiArm('local', name)
    yumi_methods = YuMiArm.__dict__

    # Define how requests are handled (Call the corresponding method in the local class)
    def handle_request(req):
        func = pickle.loads(req.func)
        args = pickle.loads(req.args)
        kwargs = pickle.loads(req.kwargs)
        if verbose:
            print("Handling request to call method {0} for {1} arm".format(func, name))
        return ROSYumiArmResponse(pickle.dumps(yumi_methods[func](arm, *args, **kwargs)))
    
    # Initialize server and service
    rospy.init_node('{}_arm'.format(name))
    s = rospy.Service('{}_arm'.format(name), ROSYumiArm, handle_request)
    print("{} arm is ready".format(name))

    # Keep process alive
    rospy.spin()
        