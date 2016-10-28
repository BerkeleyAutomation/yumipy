from serial import Serial

from yumi_constants import YuMiConstants as YMC

class ResetDevice:

    @staticmethod
    def _cmd(cmd, comm, baudrate, blocking):
        ser = Serial(comm,baudrate)
        ser.flushInput()
        ser.flushOutput()
        ser.write(cmd)
        _res_char = ser.readline()
        ser.close()

    @staticmethod
    def reset(comm=YMC.RESET_RIGHT_COMM, baudrate=YMC.RESET_BAUDRATE, blocking=True):
        ResetDevice._cmd('r', comm, baudrate, blocking)

    @staticmethod
    def go_high(comm=YMC.RESET_RIGHT_COMM, baudrate=YMC.RESET_BAUDRATE, blocking=True):
        ResetDevice._cmd('h', comm, baudrate, blocking)

    @staticmethod
    def go_low(comm=YMC.RESET_RIGHT_COMM, baudrate=YMC.RESET_BAUDRATE, blocking=True):
        ResetDevice._cmd('l', comm, baudrate, blocking)