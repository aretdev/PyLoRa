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

import SX127x

SX127x.BOARD.setup()


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

    def __init__(self, DEBUG=False):

        self.DEBUG = DEBUG

        self.lora = SX127x.LoRa(verbose=False,
                                do_calibration=True,
                                calibration_freq=868,
                                sf=7,
                                cr=SX127x.CODING_RATE.CR4_5,
                                freq=869)

        self.lora.set_mode(SX127x.MODE.STDBY)
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

    def __unpack(self, packet):
        header = packet[:self.HEADER_SIZE]
        content = packet[self.HEADER_SIZE:]

        sender_addr, destination_addr, flags, check = struct.unpack(self.HEADER_FORMAT, header)
        seqnum = self.ONE if (flags & 1) & 1 else self.ZERO
        acknum = self.ONE if (flags >> 2) & 1 else self.ZERO
        is_last = (flags >> 4) & 1 == 1
        pkt_type = (flags >> 6) & 1 == 1
        if (content == b''):
            payload = b''
        else:
            payload = content

        return sender_addr, destination_addr, seqnum, acknum, pkt_type, is_last, check, payload

    def __get_checksum(self, data):

        if self.DEBUG: print("DEBUG >> Data Before Getting Checksum: ", data)
        h = hashlib.sha256(data)
        ha = binascii.hexlify(h.digest())

        if self.DEBUG: print("DEBUG >> Checksum: ", ha[-3:])

        return ha[-3:]

    def _send(self, content, lora_obj, sender_addr, receiver_addr):

        sender_addr = sender_addr[8:]
        receiver_addr = receiver_addr[8:]
        if self.DEBUG: print("DEBUG >> Sender, Receiver ", sender_addr, receiver_addr)

        if len(content) == 0: print("WARNING ON SEND!: Content size is 0! continuing... ")

        # Packets to send
        totptbs = int(len(content) / self.PAYLOAD_SIZE)
        # In case it is not an integer , we add 1
        if (len(content) % self.PAYLOAD_SIZE) != 0: totptbs += 1

        if self.DEBUG: print("DEBUG >> Total pckts to send:  ", totptbs)

        # Initial timeout
        timeout_value = 5

        # STATS COUNTERS
        FAILED = 0
        stats_psent = 0
        stats_retrans = 0

        # RTT ESTIMATIONS
        timeout_time = 1
        estimated_rtt = -1
        dev_rtt = 1

        # Loractp is stop and wait protocol
        seqnum = self.ZERO
        acknum = self.ONE

        # Enable GC
        gc.enable()
        gc.collect()

        for p in range(totptbs):

            if self.DEBUG: print("DEBUG >> Processing packet:  ", p)
            last_pkt = True if (p == (totptbs - 1)) else False

            # Getting a block to sent from content
            blocktbs = content[0:self.PAYLOAD_SIZE]
            content = content[self.PAYLOAD_SIZE:]

            packet = self.__make_pkt(sender_addr, receiver_addr, seqnum, acknum, self.ITS_DATA_PKT, last_pkt, blocktbs)

            if self.DEBUG: print("DEBUG >> Sending packet:  ", packet)

            keep_trying = 3

            while keep_trying > 0:

                try:
                    time.sleep(3 - keep_trying)
                    send_time = time.time()
                    lora_obj.send(packet)
                    lora_obj.recv()
                    if self.DEBUG: print("DEBUG >> Waiting for ack...")
                    lora_obj.set_timeout(timeout_value)
                    recv_time = time.time()
                    if self.DEBUG: print("DEBUG >> Ack received!")
                    ack = bytes(lora_obj.payload)
                    ack_saddr, ack_daddr, ack_seqnum, ack_acknum, ack_is_ack, ack_final, ack_check, ack_content = \
                        self.__unpack(ack)
                    if receiver_addr == self.ANY_ADDR or receiver_addr == b'':
                        # Who sent the ACK on Any address is now the one who is going to receive packets
                        receiver_addr = ack_saddr

                    if ack_is_ack and ack_acknum == seqnum and sender_addr == ack_daddr and receiver_addr == ack_saddr:
                        stats_psent += 1
                        # No more need to retry
                        break
                    else:
                        # Received packet not valid
                        if self.DEBUG: print("ERROR: ACK received not valid")

                except TimeoutError:
                    if self.DEBUG: print("DEBUG >> ERROR! no ack received")

                if self.DEBUG: print("DEBUG >> TRYING ATTEMPT:  ", 11 - keep_trying)

                stats_psent += 1
                stats_retrans += 1
                keep_trying -= 1

                if keep_trying == 0:
                    FAILED = -1
                    break

            # If last packet or sending has failed
            if last_pkt or (FAILED < 0): break

            sample_rtt = recv_time - send_time

            if estimated_rtt == -1:
                estimated_rtt = sample_rtt
            else:
                estimated_rtt = estimated_rtt * 0.875 + sample_rtt * 0.125
            dev_rtt = 0.75 * dev_rtt + 0.25 * abs(sample_rtt - estimated_rtt)
            timeout_value = (estimated_rtt + 4 * dev_rtt)

            if self.DEBUG: print("DEBUG >> setting timeout to", estimated_rtt + 4 * dev_rtt)

            # Increment sequence and ack numbers
            seqnum = (seqnum + self.ONE) % 2  # self.ONE if seqnum == self.ZERO else self.ZERO
            acknum = (acknum + self.ONE) % 2  # self.ONE if acknum == self.ZERO else self.ZERO

        if self.DEBUG: print("DEBUG >> RETURNING tsend")
        if self.DEBUG: print("DEBUG >> Retrans: ", stats_retrans)

        # KN: Enabling garbage collection
        gc.enable()
        gc.collect()
        content = ""
        blocktbs = []
        content = []
        packet = ""
        return receiver_addr, stats_psent, stats_retrans, FAILED

    def sendit(self, content):
        msg = "test"
        msg = str.encode(msg)
        self._send(msg, self.lora, self.my_addr, self.ANY_ADDR)
