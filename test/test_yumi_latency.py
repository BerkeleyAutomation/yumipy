import logging
import argparse
import os
import time

import numpy as np
import matplotlib.pyplot as plt

from yumipy import YuMiRobot
from yumipy import YuMiConstants as YMC

targets = [YMC.L_HOME_STATE, YMC.L_RAISED_STATE, YMC.L_FORWARD_STATE]
def move_times(move_method, targets, n):
    total_times = []
    motion_times = []
    for _ in range(n):
        for target in targets:
            start = time.time()
            res = move_method(target, wait_for_res=True)
            motion_times.append(res['time'])
            end = time.time()
            total_times.append(end - start)
    total_times = np.array(total_times)
    motion_times = np.array(motion_times)
    diff_times = total_times - motion_times
    return total_times, motion_times, diff_times

def run(arm, n, v, zone, output_path):
    
    y = YuMiRobot(log_state_histories=True)
    y.set_v(v)
    y.set_z(zone)
    y_arm = getattr(y, arm)
    
    #collect data
    t, m, d = move_times(y_arm.goto_state, targets, n)
    
    #save data
    np.savez(os.path.join(output_path, 'total_times'), t)
    np.savez(os.path.join(output_path, 'motion_times'), m)
    np.savez(os.path.join(output_path, 'diff_times'), d)
    
    x_range = np.arange(len(t))
    fig = plt.figure(figsize=(12,8))
    ax = fig.gca()
    
    ax.plot(x_range, t, 'r-', label='Total times')
    ax.plot(x_range, m, 'g-', label='Motion Times')
    ax.plot(x_range, d, 'b-', label='Latency Times')

    legend = ax.legend(loc='best', shadow=False)
    frame = legend.get_frame()

    for label in legend.get_texts():
        label.set_fontsize('large')
    for label in legend.get_lines():
        label.set_linewidth(1.5)

    ax.set_title("YuMi Command Times", fontsize=20)
    ax.set_xlabel("Command", fontsize=14)
    ax.set_ylabel("Seconds", fontsize=14)
    fig.savefig(os.path.join(output_path, 'yumi_comm_times.pdf'), format='pdf')

    #histograms for all 3 times
    for data, name in ((t, 'total_times'), (m, 'motion_times'), (d, 'latencies')):
        fig = plt.figure(figsize=(12,8))
        ax = fig.gca()
        
        mean = np.mean(data)
        std = np.std(data)
        stats_str = 'mean: {:.3g}\nstd: {:.3g}'.format(mean, std)
    
        props = dict(facecolor='white', alpha=0.5)
        ax.set_title('Histogram of 2-way YuMi Communication Times: {0}'.format(name), fontsize=20)
        ax.set_xlabel('Seconds', fontsize=18)
        ax.set_ylabel('Normalized Count', fontsize=18)

        # place a text box in upper left in axes coords
        ax.text(0.05, 0.95, stats_str, transform=ax.transAxes, fontsize=14,
                verticalalignment='top', bbox=props)
        h = plt.hist(data, normed=True, bins=30)

        fig.savefig(os.path.join(output_path, 'hist_yumi_2_way_{0}.pdf'.format(name)), format='pdf')    

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.DEBUG)
    parser = argparse.ArgumentParser(description='YuMi latency data analysis')
    parser.add_argument('n', type=int, help='number of test paths the data is recorded through')
    parser.add_argument('output_path', help='the output path of data recorded')
    parser.add_argument('-v', '--v', type=int, default=400, help='speed of arm')
    parser.add_argument('-z', '--zone', type=str, default='fine', help='zone of arm')
    parser.add_argument('-a', '--arm', default='left', help='which arm the data is collected on')
    args = parser.parse_args()
    
    if args.arm not in ('left', 'right'):
        logging.error("Only left and right can be values for arm!")
        exit(0)
        
    if not os.path.exists(args.output_path):
        os.makedirs(args.output_path)
    
    run(args.arm, args.n, args.v, args.zone, args.output_path)