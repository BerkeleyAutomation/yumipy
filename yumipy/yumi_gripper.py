'''
Interface of robotic control over ethernet. Built for the YuMi
Author: Jacky Liang
'''
from multiprocessing import Queue
from Queue import Empty
import logging, socket, sys, os

from time import sleep, time
from yumi_constants import YuMiConstants as YMC
from yumi_exceptions import YuMiCommException,YuMiControlException
from yumi_util import iter_to_str, construct_req
from yumi_ethernet import _YuMiEthernet, _REQ_PACKET, _RES

class YuMiGripper:
    """ Interface to a single arm of an ABB YuMi robot.
    Communicates with the robot over Ethernet.
    """

    def __init__(self, name, motion_timeout=YMC.MOTION_TIMEOUT,
                comm_timeout=YMC.COMM_TIMEOUT, process_timeout=YMC.PROCESS_TIMEOUT,
                 debug=YMC.DEBUG):
        '''Initializes a YuMiGripper interface. This interface will communicate with one arm (port) on the YuMi Robot.
        This uses a subprocess to handle non-blocking socket communication with the RAPID server.

        Parameters
        ----------
            name : string
                    Name of the arm {'left', 'right'}
            motion_timeout : float, optional
                    Timeout for motion commands.
                    Default from YuMiConstants.MOTION_TIMEOUT
            comm_timeout : float, optional
                    Timeout for non-motion ethernet communication.
                    Default from YuMiConstants.COMM_TIMEOUT
            process_timeout : float, optional
                    Timeout for ethernet process communication.
                    Default from YuMiConstants.PROCESS_TIMEOUT
            debug : bool, optional
                    Boolean to indicate whether or not in debug mode. If in debug mode no ethernet communication is attempted. Mock responses will be returned.
                    Default to YuMiConstants.DEBUG
        '''
        self._motion_timeout = motion_timeout
        self._comm_timeout = comm_timeout
        self._process_timeout = process_timeout
        self._ip = YMC.IP

        if name == 'left':
            self._port = YMC.PORTS['left']['gripper']
        elif name == 'right':
            self._port = YMC.PORTS['right']['gripper']
        else:
            raise ValueError("Name can only be left or right! Got {0}".format(name))
        self._bufsize = YMC.BUFSIZE
        self._debug = debug

        self._name = name
        self._create_yumi_ethernet()

    def reset(self):
        '''Resets the underlying yumi ethernet process and socket, and resets all the settings.
        '''
        # terminate ethernet
        try:
            self._yumi_ethernet.terminate()
        except Exception:
            pass

        # start ethernet comm
        self._create_yumi_ethernet()

    def _create_yumi_ethernet(self):
        self._req_q = Queue()
        self._res_q = Queue()

        self._yumi_ethernet = _YuMiEthernet("griper_{0}".format(self._name), self._req_q,
                                            self._res_q, self._ip, self._port, self._bufsize,
                                            self._comm_timeout, self._debug)
        self._yumi_ethernet.start()

    def stop(self):
        '''Stops subprocess for ethernet communication. Allows program to exit gracefully.
        '''
        self._req_q.put("stop")
        try:
            self._yumi_ethernet.terminate()
        except Exception:
            pass

    def __del__(self):
        self.stop()

    def _request(self, req, wait_for_res, timeout=None):
        if timeout is None:
            timeout = self._comm_timeout

        req_packet = _REQ_PACKET(req, timeout, wait_for_res)
        logging.debug('Process req: {0}'.format(req_packet))

        self._req_q.put(req_packet)
        if wait_for_res:
            try:
                res = self._res_q.get(block=True, timeout=self._process_timeout)
            except (IOError, Empty):
                raise YuMiCommException("Request timed out: {0}".format(req_packet))

            logging.debug('res: {0}'.format(res))

            if res.res_code != YMC.RES_CODES['success']:
                raise YuMiControlException(req_packet, res)

            return res

    def ping(self, wait_for_res=True):
        '''Pings the remote server.
        Parameters
        ----------
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        out : Namedtuple (raw_res, data) from ping command.

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        req = construct_req('ping')
        return self._request(req, wait_for_res)

    def open(self, no_wait=False, wait_for_res=True):
        '''Opens the gripper to the target_width
        Parameters
        ----------
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        None if wait_for_res is False
        namedtuple('_RAW_RES', 'mirror_code res_code message') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        YuMiControlException
            If commanded pose triggers any motion errors that are catchable by RAPID sever.
        '''
        req = construct_req('open_gripper', '')
        return self._request(req, wait_for_res, timeout=self._motion_timeout)

    def close(self, wait_for_res=True):
        '''Closes the gripper as close to 0 as possible with maximum force.
        Parameters
        ----------
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        None if wait_for_res is False
        namedtuple('_RAW_RES', 'mirror_code res_code message') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        YuMiControlException
            If commanded pose triggers any motion errors that are catchable by RAPID sever.
        '''
        req = construct_req('close_gripper', '')
        return self._request(req, wait_for_res, timeout=self._motion_timeout)

    def move(self, width, no_wait=False, wait_for_res=True):
        '''Moves the gripper to the given width in meters.
        Parameters
        ----------
        width : float
                Target width in meters
        no_wait : bool, optional
                If True, the RAPID server will continue without waiting for the gripper to reach its target width
                Defaults to False
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        None if wait_for_res is False
        namedtuple('_RAW_RES', 'mirror_code res_code message') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        YuMiControlException
            If commanded pose triggers any motion errors that are catchable by RAPID sever.
        '''
        lst = [width * YMC.METERS_TO_MM]
        if no_wait:
            lst.append(0)
        body = iter_to_str('{0:.1f}', lst)
        req = construct_req('move_gripper', body)
        return self._request(req, wait_for_res, timeout=self._motion_timeout)

    def calibrate(self, max_speed=None, hold_force=None, phys_limit=None, wait_for_res=True):
        '''Calibrates the gripper.
        Parameters
        ----------
        max_speed : float, optional
                Max speed of the gripper in mm/s.
                Defaults to None. If None, will use maximum speed in RAPID.
        hold_force : float, optional
                Hold force used by the gripper in N.
                Defaults to None. If None, will use maximum force the gripper can provide (20N).
        phys_limit : float, optional
                The maximum opening of the gripper.
                Defaults to None. If None, will use maximum opening the gripper can provide (25mm).
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        None if wait_for_res is False
        namedtuple('_RAW_RES', 'mirror_code res_code message') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.

        Notes
        -----
        All 3 values must be provided, or they'll all default to None.
        '''
        if None in (max_speed, hold_force, phys_limit):
            body = ''
        else:
            body = self._iter_to_str('{:.1f}', [data['max_speed'], data['hold_force'], data['phys_limit']])
        req = construct_req('calibrate_gripper', body)
        return self._request(req, wait_for_res, timeout=self._motion_timeout)

    def set_force(self, force, wait_for_res=True):
        '''Sets the gripper hold force
        Parameters
        ----------
        force : float
                Hold force by the gripper in N.
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        None if wait_for_res is False
        namedtuple('_RAW_RES', 'mirror_code res_code message') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        body = self._iter_to_str('{:.1f}', [force])
        req = construct_req('set_gripper_force', body)
        self._last_sets['gripper_force'] = force
        return self._request(req, wait_for_res)

    def set_max_speed(self, max_speed, wait_for_res=True):
        '''Sets the gripper max speed
        Parameters
        ----------
        max_speed : float
                In mm/s.
        wait_for_res : bool, optional
                If True, will block main process until response received from RAPID server.
                Defaults to True

        Returns
        -------
        None if wait_for_res is False
        namedtuple('_RAW_RES', 'mirror_code res_code message') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        '''
        body = self._iter_to_str('{:1f}', [max_speed])
        req = construct_req('set_gripper_max_speed', body)
        self._last_sets['gripper_max_speed'] = max_speed
        return self._request(req, wait_for_res)

    def get_width(self, raw_res=False):
        '''Get width of current gripper in meters.
        Parameters
        ----------
        raw_res : bool, optional

        Returns
        -------
        Width in meters if raw_res is False
        namedtuple('_RES', 'res width') otherwise

        Raises
        ------
        YuMiCommException
            If communication times out or socket error.
        YuMiControlException
            If commanded pose triggers any motion errors that are catchable by RAPID sever.
        '''
        req = construct_req('get_gripper_width')
        res = self._request(req, wait_for_res=True)

        if self._debug:
            return -1.

        width = float(res.message) * MM_TO_METERS
        if raw_res:
            return _RES(res, width)
        else:
            return width
