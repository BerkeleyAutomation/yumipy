'''
Class for streaming pose and state information from the YuMi
Author: Jacky Liang
'''

from multiprocessing import Process, Queue
from Queue import Empty
import socket
import sys
from collections import namedtuple
import logging
from time import sleep

from yumi_constants import YuMiConstants as YMC
from yumi_state import YuMiState
from yumi_exceptions import YuMiCommException
from yumi_util import message_to_state, message_to_pose, message_to_torques

_RAW_RES = namedtuple('_RAW_RES', 'time message')

class _YuMiSubscriberEthernet(Process):

    def __init__(self, data_q, ip, port, bufsize, timeout):
        Process.__init__(self)
        self._data_q = data_q

        self._ip = ip
        self._port = port
        self._bufsize = bufsize
        self._timeout = timeout

        self._end_run = False

        self._socket = None

    def run(self):
        logging.getLogger().setLevel(YMC.LOGGING_LEVEL)

        self._reset_socket()

        try:
            while not self._end_run:
                        
                raw_res = None
                try:
                    raw_res = self._socket.recv(self._bufsize)
                except socket.error, e:
                    if e.errno == 114: # request time out
                        raise YuMiCommException('Request timed out')

                if raw_res is None or len(raw_res) == 0:
                    raise YuMiCommException('Empty response!')

                raw_res = raw_res[raw_res.rfind("#")+1:]
                tokens = raw_res.split()
                res = _RAW_RES(float(tokens[0]), ' '.join(tokens[1:]))

                if self._data_q.full():
                    try:
                        self._data_q.get_nowait()
                    except Empty:
                        pass
                self._data_q.put(res)

        except KeyboardInterrupt:
            self._stop()
            sys.exit(0)

    def _stop(self):
        logging.info("Shutting down yumi ethernet interface")
        self._socket.close()
        self._end_run = True

    def _reset_socket(self):
        logging.debug('Opening socket on {0}:{1} for subscription'.format(self._ip, self._port))
        if self._socket != None:
            self._socket.close()

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(self._timeout)
        self._socket.connect((self._ip, self._port))
        logging.debug('Socket successfully opened!')

class _YuMiArmSubscriber:

    def __init__(self, name, ip=YMC.IP, bufsize=YMC.BUFSIZE, comm_timeout=YMC.COMM_TIMEOUT):
        self._name = name
        self._to_frame = "yumi_{0}".format(name)        
        self._comm_timeout = comm_timeout
        self._bufsize = bufsize
        self._ip = ip
        self._time_offset = 0

    def _start(self):
        self._state_q = Queue(maxsize=1)
        self._pose_q = Queue(maxsize=1)
        self._torque_q = Queue(maxsize=1)

        self._sub_pose = _YuMiSubscriberEthernet(self._pose_q, self._ip, YMC.PORTS[self._name]["poses"], self._bufsize, self._comm_timeout)
        self._sub_state = _YuMiSubscriberEthernet(self._state_q, self._ip, YMC.PORTS[self._name]["joints"], self._bufsize, self._comm_timeout)
        self._sub_torque = _YuMiSubscriberEthernet(self._torque_q, self._ip, YMC.PORTS[self._name]["torques"], self._bufsize, self._comm_timeout)
        
        self._sub_pose.start()
        self._sub_state.start()
        self._sub_torque.start()

        self._last_state = None
        self._last_pose = None
        self._last_torque = None

    def _stop(self):
        self._sub_ethernet.terminate()

    def _reset_time(self):
        self._time_offset += self.get_pose()[0]

    def get_pose(self):
        '''Get the current pose of this arm.

        Returns
        -------
        out : float (seconds since start or last call to reset, whichever one is more recent), RigidTransform

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        if not self._pose_q.empty() or self._last_pose is None:
            res = self._pose_q.get(block=True)
            self._last_pose = res.time, message_to_pose(res.message, self._to_frame)
            
        time_stamp, pose = self._last_pose            
        return time_stamp - self._time_offset, pose.copy()

    def get_state(self):
        '''Get the current state (joint configuration) of this arm and corresponding timestamp.

        Returns
        -------
        out : float (seconds since start or last call to reset, whichever one is more recent), YuMiState

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        if not self._state_q.empty() or self._last_state is None:
            res = self._state_q.get(block=True)
            self._last_state = res.time, message_to_state(res.message)
        time_stamp, state = self._last_state
        return time_stamp - self._time_offset, state.copy()

    def get_torque(self):
        '''Get the current torque readings of each joint of this arm and corresponding timestamp.

        Returns
        -------
        out : float (seconds since start or last call to reset, whichever one is more recent), numpy array

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        if not self._torque_q.empty() or self._last_torque is None:
            res = self._torque_q.get(block=True)
            self._last_torque = res.time, message_to_torques(res.message)
        time_stamp, torque = self._last_torque
        return time_stamp - self._time_offset, torque.copy()

class YuMiSubscriber:

    def __init__(self, include_left=True, include_right=True):
        '''Initializes a YuMiSubscriber
        
        Parameters
        ----------
            include_left : bool, optional
                    If True, the left arm subscriber is included and instantiated.
                    Defaults to True
            include_right : bool, optional
                    If True, the right arm subscriber is included and instantiated.
                    Defaults to True

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        if not include_left and not include_right:
            raise ValueError("Must include at least one arm to subscribe to!")

        self._arms = {}
        if include_left:
            self.left = _YuMiArmSubscriber("left")
            self._arms['left'] = self.left
        if include_right:
            self.right = _YuMiArmSubscriber("right")
            self._arms['right'] = self.right
        self._started = False
        
    def stop(self):
        '''Calls the stop function for each instantiated arm subscriber object.
        '''
        if not self._started:
            raise Exception("Cannot stop a YuMiSubscriber if it hasn't started yet!")

        for sub in self._arms.values():
            sub._stop()

    def start(self):
        '''Calls the start function for each instantiated arm subscriber object.
        '''
        if self._started:
            raise Exception("Can only start a YuMiSubscriber once!")

        self._started = True
        for sub in self._arms.values():
            sub._start()

    def reset_time(self):
        if not self._started:
            raise Exception("Can only reset time if YuMiSubscriber has started!")

        for sub in self._arms.values():
            sub._reset_time()