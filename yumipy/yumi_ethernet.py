"""
Helper YuMiEthernet Class
Author: Jacky Liang
"""
from multiprocessing import Process, Queue
from setproctitle import setproctitle
from Queue import Empty
import sys, logging, socket
from collections import namedtuple
from time import sleep
from yumi_constants import YuMiConstants as YMC

_RAW_RES = namedtuple('_RAW_RES', 'mirror_code res_code message')
_RES = namedtuple('_RES', 'raw_res data')
_REQ_PACKET = namedtuple('_REQ_PACKET', 'req timeout return_res')

class _YuMiEthernet(Process):

    def __init__(self, name, req_q, res_q, ip, port, bufsize, timeout, debug):
        Process.__init__(self)

        self._name = name

        self._ip = ip
        self._port = port
        self._timeout = timeout
        self._bufsize = bufsize
        self._socket = None

        self._req_q = req_q
        self._res_q = res_q

        self._current_state = None

        self._debug = debug

    def run(self):
        setproctitle('python._YuMiEthernet.{0}'.format(self._name))
        logging.getLogger().setLevel(YMC.LOGGING_LEVEL)

        if self._debug:
            logging.info("In DEBUG mode. Messages will NOT be sent over socket.")
        else:
            self._reset_socket()

        try:
            while True:
                req_packet = self._req_q.get()
                if req_packet == "stop":
                    break
                res = self._send_request(req_packet)
                if req_packet.return_res:
                    self._res_q.put(res)

                sleep(YMC.PROCESS_SLEEP_TIME)

        except KeyboardInterrupt:
            self._stop()
            sys.exit(0)

        self._stop()

    def _stop(self):
        logging.debug("Shutting down yumi ethernet interface")
        if not self._debug:
            self._socket.close()

    def _reset_socket(self):
        logging.debug('Opening socket on {0}:{1}'.format(self._ip, self._port))
        if self._socket != None:
            self._socket.close()

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(self._timeout)
        self._socket.connect((self._ip, self._port))
        logging.debug('Socket successfully opened!')

    def _send_request(self, req_packet):
        logging.debug("Sending: {0}".format(req_packet))
        raw_res = None

        if self._debug:
            raw_res = '-1 1 MOCK RES for {0}'.format(req_packet)
        else:
            self._socket.settimeout(req_packet.timeout)

            while True:
                try:
                    self._socket.send(req_packet.req)
                    break
                except socket.error, e:
                    # TODO: better way to handle this mysterious bad file descriptor error
                    if e.errno == 9:
                        self._reset_socket()
            try:
                raw_res = self._socket.recv(self._bufsize)
            except socket.error, e:
                if e.errno == 114: # request time out
                    raise YuMiCommException('Request timed out: {0}'.format(req_packet))

        logging.debug("Received: {0}".format(raw_res))

        if raw_res is None or len(raw_res) == 0:
            raise YuMiCommException('Empty response! For req: {0}'.format(req_packet))

        tokens = raw_res.split()
        res = _RAW_RES(int(tokens[0]), int(tokens[1]), ' '.join(tokens[2:]))
        return res
