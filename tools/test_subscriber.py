"""
Demonstrate the YuMiSubscriber
Author: Jeff Mahler
"""
import IPython
import logging
import matplotlib.pyplot as plt
import numpy as np

from yumipy import YuMiSubscriber

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    subscriber = YuMiSubscriber()
    subscriber.start()

    # read torques
    cur_time = 0
    times = []
    torques = []
    while cur_time < 60.0:
        cur_time, torque = subscriber.left.get_torque()
        logging.info('Cur time: %.4f' %(cur_time))
        times.append(cur_time)
        torques.append(torque)

    # plot 
    torques = np.array(torques)
    torque_dim = torques.shape[1]
    colors = ['r', 'g', 'b', 'c', 'y', 'm', 'k']
    for i in range(torque_dim):
        plt.figure()
        plt.plot(times, torques[:,i], c=colors[i], linewidth=2)
        plt.title('Torque %d' %(i))
    plt.show()

    subscriber.stop()
