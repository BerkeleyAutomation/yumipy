#!/usr/bin/env python

import logging
try:
    from yumipy.yumi_arm import *
    from yumipy.srv import *
except ImportError:
    raise RuntimeError("yumi_ros_service unavailable outside of catkin package")

if __name__ == '__main__':
    if(len(sys.argv) != 2):
        raise RuntimeError("Incorrect number of arguments (expected 2)")
    side = sys.argv[1].lower()
    if side in {'left', 'right'}:
        arm = YuMiArmFactory.YuMiArm('local', side)
        yumi_methods = YuMiArm.__dict__

        def handle_request(req):
            func = pickle.loads(req.func)
            args = pickle.loads(req.args)
            kwargs = pickle.loads(req.kwargs)
            logging.info("Handling request to call method {0} for {1} arm".format(func, side))
            return ROSYumiArmResponse(pickle.dumps(yumi_methods[func](arm, *args, **kwargs)))
        
        def arm_server():
            rospy.init_node('{}_arm_server'.format(side))
            s = rospy.Service('{}_arm'.format(side), ROSYumiArm, handle_request)
            logging.info("{} arm is ready".format(side))

        arm_server()
        rospy.spin()
    else:
        raise RuntimeError("Arm name must be in {'left', 'right'}")