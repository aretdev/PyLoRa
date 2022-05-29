import binascii
import gc
import hashlib
import socket
import struct
import sys
import time
import hashlib
import socket
import getmac

from SX127x.LoRa import LoRa
from SX127x.board_config import BOARD
from SX127x.constants import CODING_RATE

BOARD.setup()


class CTPLoraEndPoint:
    DEBUG = False

    MAX_PKT_SIZE = 230  # 8B + 8B + 1B + 3B
    HEADER_SIZE = 20
    PAYLOAD_SIZE = MAX_PKT_SIZE - HEADER_SIZE

    HEADER_FORMAT = "!8s8sB3s"

    ITS_DATA_PKT = False
    ITS_ACK_PKT = True

    ANY_ADDR = b'\x00\x00\x00\x00\x00\x00\x00\x00'

    ONE = 1
    ZERO = 0

    def __init__(self, verbose=True, do_calibration=True, calibration_freq=868, sf=7, cr=CODING_RATE.CR4_5, freq=869):

        self.lora = LoRa(verbose, do_calibration, calibration_freq, sf, cr, freq)
        self.lora.set_pa_config(pa_select=1)

        self.lora_mac = binascii.hexlify(bytes(getmac.get_mac_address(), encoding='utf8'))
        self.my_addr = self.lora_mac[8:]

    def __make_pkt(self, source, destination, seqnum, acknum, pkt_type, is_last, payload):

        if self.DEBUG: print("DEBUG >> Making pkt: ", source, destination, seqnum, acknum, pkt_type, is_last, payload)
        #                 0/1   0/1   0/1     0/1
        # FLAGS FORMAT : [SEQ | ACK | LAST | TYPE ]
        flags = 0
        if seqnum == self.ONE:
            flags = 1
            print(" - DEBUG flags >>: ", bin(flags))

        if acknum == self.ONE:
            flags = flags | (1 << 2)
            print(" - DEBUG flags >>: ", bin(flags))

        if is_last:
            flags = flags | (1 << 4)
            print(" - DEBUG flags >>: ", bin(flags))

        if pkt_type == self.ITS_ACK_PKT:
            flags = flags | (1 << 6)
            print(" - DEBUG flags >>: ", bin(flags))

        if len(payload) > 0 and pkt_type == self.ITS_DATA_PKT:
            # I'm a data packet, getting checksum of payload!
            p = payload
            checksum = self.__get_checksum(p)
            header = struct.pack(self.HEADER_FORMAT, source, destination, flags, checksum)
            if self.DEBUG: print("DEBUG >> Resultant Data Packet: ", header + p)
        else:
            p = b''  # I'm an ACK, no data content transmitted
            header = struct.pack(self.HEADER_FORMAT, source, destination, flags, p)
            if self.DEBUG: print("DEBUG >> Resultant ACK Packet: ", header + p)
        return header + p

    def __get_checksum(self, data):

        if self.DEBUG: print("DEBUG >> Data Before Getting Checksum: ", data)
        h = hashlib.sha256(data)
        ha = binascii.hexlify(h.digest())

        if self.DEBUG: print("DEBUG >> Checksum: ", ha[-3:])

        return ha[-3:]

    def sendit(self, content):
        l = self.__make_pkt(self.ANY_ADDR, self.ANY_ADDR, self.ZERO, self.ZERO, self.ITS_DATA_PKT, False,
                            bytes([12, 2]))

        print(l)
        self.lora.send(l)
        self.lora.set_timeout(value=10000)
