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
from yumi_util import message_to_state, message_to_pose

_RAW_RES = namedtuple('_RAW_RES', 'type_code time message')

class _YuMiSubscriberEthernet(Process):

    def __init__(self, pose_q, state_q, ops_q, ip, port, bufsize, timeout):
        Process.__init__(self)
        self._pose_q = pose_q
        self._state_q = state_q
        self._ops_q = ops_q

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
                if not self._ops_q.empty():
                    op_name = self._ops_q.get()
                    attr = '_{0}'.format(op_name)
                    if hasattr(self, attr):
                        getattr(self, attr)()
                    else:
                        logging.error("Unknown op {0}. Skipping".format(op_name))
                        
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
                res = _RAW_RES(int(tokens[0]), float(tokens[1]), ' '.join(tokens[2:]))

                if res.type_code == YMC.SUB_CODES['pose']:
                    if self._pose_q.full():
                        try:
                            self._pose_q.get_nowait()
                        except Empty:
                            pass
                    self._pose_q.put(res)
                elif res.type_code == YMC.SUB_CODES['state']:
                    if self._state_q.full():
                        try:
                            self._state_q.get_nowait()
                        except Empty:
                            pass
                    self._state_q.put(res)
                else:
                    raise YuMiCommException("Unknown subscriber message type! Got {0}".format(res.type_code))

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
    def __init__(self, port, name, ip=YMC.IP, bufsize=YMC.BUFSIZE, comm_timeout=YMC.COMM_TIMEOUT):
        self._name = name
        self._to_frame = "yumi_{0}".format(name)        
        self._comm_timeout = comm_timeout
        self._bufsize = bufsize
        self._ip = ip
        self._port = port
        self._time_offset = 0

    def _start(self):
        self._state_q = Queue(maxsize=1)
        self._pose_q = Queue(maxsize=1)
        self._ops_q = Queue()

        self._sub_ethernet = _YuMiSubscriberEthernet(self._pose_q, self._state_q, self._ops_q, self._ip, self._port, self._bufsize, self._comm_timeout)
        self._sub_ethernet.start()

        self._last_state = None
        self._last_pose = None

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

class YuMiSubscriber:
    """ Interface to stream pose and state information from an ABB YuMI robot
    over ethernet.
    """

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
            self.left = _YuMiArmSubscriber(YMC.PORT_L_SUB, "left")
            self._arms['left'] = self.left
        if include_right:
            self.right = _YuMiArmSubscriber(YMC.PORT_R_SUB, "right")
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
