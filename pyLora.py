import os
from constants import *
try:
    machine = os.uname().machine
except Exception:
    machine = os.name


class pyLora:
    IS_RPi = machine.startswith('armv')
    IS_ESP8266 = machine.startswith('ESP8266')
    IS_ESP32 = machine.startswith('ESP32')
    IS_LOPY = machine.startswith('LoPy')

    __SX127X_LIB = None

    timeout_socket = None
    blocked_socket = None

    def __init__(self, verbose=False,
                 do_calibration=False,
                 calibration_freq=868,
                 sf=7,
                 cr=8,
                 freq=868):

        auto_board_selection = None

        if self.IS_RPi:
            from board_config_rpi import BOARD_RPI
            auto_board_selection = BOARD_RPI

        if self.IS_ESP32:
            from board_config_esp32 import BOARD_ESP32
            auto_board_selection = BOARD_ESP32

        if self.IS_LOPY:
            from network import LoRa
            import socket

            loraLoPyModule = LoRa(mode=LoRa.LORA, region=LoRa.EU868, frequency=freq*1000000, coding_rate=LoRa.CODING_4_5,
                        bandwidth=LoRa.BW_125KHZ, sf=sf)
            self.lopyLora = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
        else:
            from LoRa import LoRa
            self.__SX127X_LIB = LoRa(Board_specification=auto_board_selection,
                                     verbose=verbose,
                                     do_calibration=do_calibration,
                                     calibration_freq=calibration_freq,
                                     cr=CODING_RATE.CR4_5,
                                     sf=sf,
                                     freq=freq,
                                     rx_crc=True)

    def send(self, content):
        if self.IS_LOPY: return self.lopyLora.send(content)
        self.__SX127X_LIB.set_mode(MODE.SLEEP)
        self.__SX127X_LIB.set_dio_mapping([1, 0, 0, 0])  # DIO0 = 1 is for TXDone, transmitting mode basically
        self.__SX127X_LIB.set_mode(MODE.STDBY)
        formatted = list(content)
        self.__SX127X_LIB.write_payload(formatted)  # I send my payload to LORA SX1276 interface
        self.__SX127X_LIB.set_mode(MODE.TX)  # I enter on TX Mode
        self.__SX127X_LIB.set_dio0_status(timeout_value=self.timeout_socket, socket_blocked=self.blocked_socket)

    def recv(self, size=230):
        """ Util Method for recv
            It will turn automatically the device on receive mode
        """
        if self.IS_LOPY: return self.lopyLora.recv(size)
        self.__SX127X_LIB.set_mode(MODE.SLEEP)
        self.__SX127X_LIB.set_dio_mapping([0, 0, 0, 0])
        self.__SX127X_LIB.set_mode(MODE.RXCONT)
        self.__SX127X_LIB.set_dio0_status(timeout_value=self.timeout_socket, socket_blocked=self.blocked_socket)
        return bytes(self.__SX127X_LIB.payload)

    def settimeout(self, value):
        """ set timeout for operations
            After we determine if we want to send or receive, we need to specify a timeout
        """
        if self.IS_LOPY: return self.lopyLora.settimeout(value)
        self.timeout_socket = value


    def setblocking(self, value):
        if self.IS_LOPY: return self.lopyLora.setblocking(value)
        self.blocked_socket = value
