import os
from LoRa import LoRa
from constants import *

try:
    machine = os.uname().machine
except Exception:
    machine = os.name


class pyLora:
    IS_RPi = machine.startswith('armv')
    IS_ESP8266 = machine.startswith('ESP8266')
    IS_ESP32 = machine.startswith('ESP32')

    __SX127X_LIB = None

    timeout_socket = None
    blocked_socket = None

    def __init__(self, verbose=False,
                 do_calibration=True,
                 calibration_freq=868,
                 sf=7,
                 cr=CODING_RATE.CR4_5,
                 freq=868):

        auto_board_selection = None

        if self.IS_RPi:
            from board_config_rpi import BOARD_RPI
            auto_board_selection = BOARD_RPI

        if self.IS_ESP32:
            from board_config_esp32 import BOARD_ESP32
            auto_board_selection = BOARD_ESP32

        self.__SX127X_LIB = LoRa(Board_specification=auto_board_selection,
                                 verbose=verbose,
                                 do_calibration=do_calibration,
                                 calibration_freq=calibration_freq,
                                 cr=cr,
                                 sf=sf,
                                 freq=freq)

        print(self.__SX127X_LIB)

    def send(self, content):
        self.__SX127X_LIB.set_mode(MODE.SLEEP)
        self.__SX127X_LIB.set_dio_mapping([1, 0, 0, 0])  # DIO0 = 1 is for TXDone, transmitting mode basically
        self.__SX127X_LIB.set_mode(MODE.STDBY)
        formatted = list(content)
        self.__SX127X_LIB.write_payload(formatted)  # I send my payload to LORA SX1276 interface
        self.__SX127X_LIB.set_mode(MODE.TX)  # I enter on TX Mode
        self.__SX127X_LIB.set_dio0_status(timeout_value=self.timeout_socket, socket_blocked=self.blocked_socket)

    def recv(self, size):
        """ Util Method for recv
            It will turn automatically the device on receive mode
        """
        self.__SX127X_LIB.set_payload_length(size)
        self.__SX127X_LIB.set_mode(MODE.SLEEP)
        self.__SX127X_LIB.set_dio_mapping([0, 0, 0, 0])
        self.__SX127X_LIB.reset_ptr_rx()
        self.__SX127X_LIB.set_mode(MODE.RXCONT)
        self.__SX127X_LIB.set_dio0_status(timeout_value=self.timeout_socket, socket_blocked=self.blocked_socket)
        return bytes(self.__SX127X_LIB.payload)

    def settimeout(self, value):
        """ set timeout for operations
            After we determine if we want to send or receive, we need to specify a timeout
        """
        self.timeout_socket = value

    def get_rssi_pkt(self):
        return self.__SX127X_LIB.get_pkt_rssi_value()

    def setblocking(self, value):
        self.blocked_socket = value
