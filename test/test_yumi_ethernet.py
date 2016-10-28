'''
Script to test connection to YuMi using YuMiEthernet interface
Authors: Jeff, Jacky
'''

import logging
import time
import os
import unittest
import numpy as np
import copy
import sys

from yumipy import YuMiConstants as YMC
from yumipy import YuMiRobot, YuMiState
from core import RigidTransform

import IPython

class YuMiServerTest(unittest.TestCase):

    @classmethod
    def setUpClass(test_class):
        test_class.setup_yumi_ethernet()

    @classmethod
    def tearDownClass(test_class):
        test_class.stop_yumi_ethernet()

    @staticmethod
    def setup_yumi_ethernet():
        YuMiServerTest.yumi = YuMiRobot(include_right=False, log_state_histories=True, log_pose_histories=True)

    @staticmethod
    def stop_yumi_ethernet():
        YuMiServerTest.yumi.reset_home()
        YuMiServerTest.yumi.stop()

    def test_ping(self):
        """ Test basic connectivity to the YuMi rapid server """
        res = YuMiServerTest.yumi.left.ping()
        self.assertEqual(res.res_code, 1)
        
    def test_command_pose(self):
        """ Test sending pose to the YuMi rapid server """
        YuMiServerTest.yumi.set_v(200)
        hist = YuMiServerTest.yumi.left.goto_pose(YMC.L_HOME_POSE)
        self.assertEqual(hist['res'].res_code, 1)
        hist = YuMiServerTest.yumi.left.goto_pose(YMC.L_FORWARD_POSE)
        self.assertEqual(hist['res'].res_code, 1)
        hist = YuMiServerTest.yumi.left.goto_pose(YMC.L_HOME_POSE)
        self.assertEqual(hist['res'].res_code, 1)

    def test_command_joints(self):
        """ Test sending to the YuMi rapid server """       
        YuMiServerTest.yumi.set_v(200) 
        hist = YuMiServerTest.yumi.left.goto_state(YMC.L_HOME_STATE)
        self.assertEqual(hist['res'].res_code, 1)
        hist = YuMiServerTest.yumi.left.goto_state(YMC.L_FORWARD_STATE)
        self.assertEqual(hist['res'].res_code, 1)
        hist = YuMiServerTest.yumi.left.goto_state(YMC.L_HOME_STATE)
        self.assertEqual(hist['res'].res_code, 1)
        
    def test_read_pose(self):
        """ Test sending to the YuMi rapid server """
        pose = YuMiServerTest.yumi.left.get_pose()
        self.assertEqual(type(pose), RigidTransform)
        logging.info('YuMi is in pose {0}'.format(pose))
        
    def test_read_joints(self):
        """ Test sending to the YuMi rapid server """
        state = YuMiServerTest.yumi.left.get_state()
        self.assertTrue(isinstance(state, YuMiState))
        logging.info('YuMi is in state {0}'.format(state))

    def _get_circular_poses(self):
        """ Moves the YuMi end effector in a small circle """
        pose_instruction_code = 1
        num_theta = 10
        num_circles = 1
        circle_radius = 0.05
        center_pose = YMC.L_FORWARD_POSE

        all_poses = []
        for j in range(num_circles):
            for i in range(num_theta):
                cur_theta = 2 * np.pi * float(i) / float(num_theta)
                pose = copy.copy(center_pose)
                pose._translation[0] += circle_radius * np.cos(cur_theta)
                pose._translation[1] += circle_radius * np.sin(cur_theta)
                all_poses.append(pose.copy())

        return all_poses
        
    def test_fast_pose_sequence(self):
        self.yumi.set_v(800)
        self.yumi.set_z('z100')
        for pose in self._get_circular_poses():
            pose_start = time.time()
            YuMiServerTest.yumi.left.goto_pose(pose, wait_for_res=True)
            pose_stop = time.time()
            logging.info('Pose command latency was %f sec' %(pose_stop - pose_start))
            
    def test_fast_pose_linear_buffer(self):
        start = time.time()
        all_poses = self._get_circular_poses()
        self.yumi.left.buffer_clear()
        self.yumi.left.buffer_add_all(all_poses)
        self.yumi.set_v(800)
        self.yumi.set_z('z100')
        self.yumi.left.buffer_move()
        end = time.time()
        dur = end - start
        logging.info("Circular move took {0}s in total, about {1}s per waypoint".format(dur, dur /1./ len(all_poses)))

if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    test_suite = unittest.TestSuite()
    test_suite.addTest(YuMiServerTest('test_ping'))
    test_suite.addTest(YuMiServerTest('test_read_pose'))
    test_suite.addTest(YuMiServerTest('test_read_joints'))
    test_suite.addTest(YuMiServerTest('test_command_joints'))
    test_suite.addTest(YuMiServerTest('test_command_pose'))    
    test_suite.addTest(YuMiServerTest('test_fast_pose_linear_buffer'))  
    test_suite.addTest(YuMiServerTest('test_fast_pose_sequence'))   
    #test_suite = unittest.TestLoader().loadTestsFromTestCase(YuMiServerTest)
    unittest.TextTestRunner(verbosity=2).run(test_suite)
