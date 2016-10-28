from yumipy import YuMiConstants as YMC
from yumipy import YuMiSubscriber

import logging
from time import sleep

if __name__ == "__main__":
    logging.getLogger().setLevel(YMC.LOGGING_LEVEL)
    sub = YuMiSubscriber()    
    sub.start()

    counter = 0
    while True:
        sleep(0.1)
        if counter % 10 == 0:
        	print "reseting time"
        	sub.reset_time()

        t1, pose_l = sub.left.get_pose()
        t2, pose_r = sub.right.get_pose()

        logging.info("{0} L: {1}| {2} R: {3}".format(t1, pose_l.translation, t2, pose_r.translation))
        counter += 1