#!/usr/bin/env python

import logging
try:
    from yumipy.yumi_arm import *
    from yumipy.srv import *
except ImportError:
    raise RuntimeError("yumi_ros_service unavailable outside of catkin package")

if __name__ == '__main__':
    # Check that the "arm side" argument is passed in and is either "left" or "right"
    if(len(sys.argv) < 2):
        raise RuntimeError("Not enough arguments (expected at least 1 for arm side)")
    side = sys.argv[1].lower()
    if side not in {'left', 'right'}:
        raise RuntimeError("Arm name must be in {'left', 'right'}")
    
    # Get local YuMiArm and its method dict
    arm = YuMiArmFactory.YuMiArm('local', side)
    yumi_methods = YuMiArm.__dict__

    # Define how requests are handled (Call the corresponding method in the local class)
    def handle_request(req):
        func = pickle.loads(req.func)
        args = pickle.loads(req.args)
        kwargs = pickle.loads(req.kwargs)
        logging.info("Handling request to call method {0} for {1} arm".format(func, side))
        return ROSYumiArmResponse(pickle.dumps(yumi_methods[func](arm, *args, **kwargs)))
    
    # Initialize server and service
    rospy.init_node('{}_arm_server'.format(side))
    s = rospy.Service('{}_arm'.format(side), ROSYumiArm, handle_request)
    logging.info("{} arm is ready".format(side))

    # Keep process alive
    rospy.spin()
        