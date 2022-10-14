import binascii
import gc
import hashlib
import socket
import struct
import sys
import time
import hashlib
import socket
import os

from board_config import BOARD
from pyLora import pyLora


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

        self.lora = pyLora(verbose=True, sf=7)
        # from network import LoRa
        # import socket
        #
        # loraLoPyModule = LoRa(mode=LoRa.LORA, region=LoRa.EU868, frequency=868*1000000, coding_rate=LoRa.CODING_4_5,
        #                 bandwidth=LoRa.BW_125KHZ, sf=7, power_mode = LoRa.ALWAYS_ON, tx_retries = 1, tx_power = 14, preamble = 8)
        # self.lora  = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

        self.lora_mac = b'e39c7a1c'
        self.my_addr = self.lora_mac[8:]

    def __make_pkt(self, source, destination, seqnum, acknum, pkt_type, is_last, payload):

        if self.DEBUG: print("DEBUG >> Making pkt: ", source, destination, seqnum, acknum, pkt_type, is_last, payload)
        #                 0/1   0/1   0/1     0/1
        # FLAGS FORMAT : [SEQ | ACK | LAST | TYPE ]
        flags = 0
        if seqnum == self.ONE:
            flags = flags | (1 << 0)
        if acknum == self.ONE:
            flags = flags | (1 << 2)
        if is_last:
            flags = flags | (1 << 4)
        if pkt_type == self.ITS_ACK_PKT:
            flags = flags | (1 << 6)

        if (len(payload) > 0 and (pkt_type == self.ITS_DATA_PKT)):
            # p = struct.pack(self.PAYLOAD_FORMAT, content)
            p = payload
            check = self.__get_checksum(p)
            h = struct.pack(self.HEADER_FORMAT, source, destination, flags, check)
            if self.DEBUG: print("DEBUG 096:", h + p)
        else:
            p = b''
            h = struct.pack(self.HEADER_FORMAT, source, destination, flags, b'')
            if self.DEBUG: print("DEBUG 100:", h + p)

        return h + p

    def __unpack(self, packet):
        header = packet[:self.HEADER_SIZE]
        content = packet[self.HEADER_SIZE:]

        sp, dp, flags, check = struct.unpack(self.HEADER_FORMAT, header)
        seqnum = self.ONE if ((flags) & 1) & 1 else self.ZERO
        acknum = self.ONE if (flags >> 2) & 1 else self.ZERO
        is_last = (flags >> 4) & 1 == 1
        pkt_type = (flags >> 6) & 1 == 1
        if (content == b''):
            payload = b''
        else:
            payload = content
        if self.DEBUG: print("DEBUG >> Unpkt: ", sp, dp, seqnum, acknum, pkt_type, is_last, payload)

        return sp, dp, seqnum, acknum, pkt_type, is_last, check, payload

    def __get_checksum(self, data):

        if self.DEBUG: print("DEBUG >> Data Before Getting Checksum: ", data)
        h = hashlib.sha256(data)
        ha = binascii.hexlify(h.digest())

        if self.DEBUG: print("DEBUG >> Checksum: ", ha[-3:])

        return (ha[-3:])

    def _csend(self, payload, the_sock, sndr_addr, rcvr_addr):

        # Shortening addresses to last 8 bytes to save space in packet
        saved_size = payload
        if self.DEBUG: print ("DEBUG 148: sndr_addr, rcvr_addr", sndr_addr, rcvr_addr)

        # computing payload (content) size as "totptbs" = total packets to be sent
        # if (len(payload)==0): print ("WARNING csend: payload size == 0... continuing")
        if (payload == 0): print("WARNING csend: payload size == 0... continuing")
        #totptbs = int(len(payload) / self.PAYLOAD_SIZE)
        totptbs = int(payload / self.PAYLOAD_SIZE)
        #if ((len(payload) % self.PAYLOAD_SIZE)!=0): totptbs += 1
        if ((payload % self.PAYLOAD_SIZE) != 0): totptbs += 1

        if self.DEBUG: print ("DEBUG 155: Total packages to be send: ", totptbs)  ###
        timeout_value = 10

        # Initialize stats counters
        FAILED         = 0
        stats_psent    = 0
        stats_retrans  = 0

        # RTT estimators
        timeout_time   =  1    # 1 second
        estimated_rtt  = -1
        dev_rtt        =  1

        # stop and wait
        seqnum = self.ZERO
        acknum = self.ONE

        # Enabling garbage collection
        gc.enable()
        gc.collect()
        start_send_t0 = time.time()
        for cp in range(totptbs):

            if self.DEBUG: print ("DEBUG 178: Packet counter: ", cp)  ###
            last_pkt = True if (cp == (totptbs-1)) else False

            # Getting a block of max self.PAYLOAD_SIZE from "payload"
            blocktbs = bytes(self.PAYLOAD_SIZE - 1)#payload[0:self.PAYLOAD_SIZE]  # Taking self.PAYLOAD_SIZE bytes ToBeSent
            #payload  = payload[self.PAYLOAD_SIZE:]   # Shifting the input string

            packet = self.__make_pkt(sndr_addr, rcvr_addr, seqnum, acknum, self.ITS_DATA_PKT, last_pkt, blocktbs)

            # trying 3 times
            keep_trying = 3
            while (keep_trying > 0):

                try:
                    time.sleep((3-keep_trying)) ###
                    the_sock.setblocking(True)
                    send_time = time.time()
                    the_sock.send(packet)
                    the_sock.settimeout(timeout_value)  ###
                    if self.DEBUG: print("DEBUG 200: waiting ACK")
                    ack = the_sock.recv(230)
                    recv_time = time.time()
                    if self.DEBUG: print("DEBUG 203: received ack", ack)
                    # self.__unpack packet information
                    ack_saddr, ack_daddr, ack_seqnum, ack_acknum, ack_is_ack, ack_final, ack_check, ack_content = self.__unpack(ack)
                    # rint("ack_saddr {}, ack_daddr {}, ack_seqnum {}, ack_acknum {}, ack_is_ack {}, ack_final {}, ack_check {}, ack_content {}".format(ack_saddr, ack_daddr, ack_seqnum, ack_acknum, ack_is_ack, ack_final, ack_check, ack_content))
                    if (rcvr_addr == self.ANY_ADDR) or (rcvr_addr == b''):
                        rcvr_addr = ack_saddr       # in case rcvr_addr was self.ANY_ADDR and payload needs many packets

                    # Check if valid...
                    if (ack_is_ack) and (ack_acknum == seqnum) and (sndr_addr == ack_daddr) and (rcvr_addr == ack_saddr):
                        stats_psent   += 1
                        # No more need to retry
                        break
                    else:
                        # Received packet not valid
                        if self.DEBUG: print ("ERROR: ACK received not valid")
                except (BOARD.LoRaTimeoutError, Exception) as e:
                    if self.DEBUG: print("EXCEPTION!! Socket timeout: ", time.time(), e)

                if self.DEBUG: print ("DEBUG 222: attempt number: ", keep_trying)
                stats_psent   += 1
                stats_retrans += 1
                keep_trying   -= 1
                if(keep_trying == 0):
                    FAILED = -1
                    break

            # Check if last packet or failed to send a packet...
            if last_pkt or (FAILED<0): break

            # RTT calculations
            sample_rtt = recv_time - send_time
            if estimated_rtt == -1:
                estimated_rtt = sample_rtt
            else:
                estimated_rtt = estimated_rtt * 0.875 + sample_rtt * 0.125
            dev_rtt = 0.75 * dev_rtt + 0.25 * abs(sample_rtt - estimated_rtt)
            timeout_value = (estimated_rtt + 4 * dev_rtt)
            if self.DEBUG: print ("241: setting timeout to", estimated_rtt + 4 * dev_rtt)

            # Increment sequence and ack numbers
            seqnum = (seqnum + self.ONE) % 2    # self.ONE if seqnum == self.ZERO else self.ZERO
            acknum = (acknum + self.ONE) % 2    # self.ONE if acknum == self.ZERO else self.ZERO

        end_send_t0 = time.time()
        difference_t0 = end_send_t0 - start_send_t0

        print ("DEBUG 247: RETURNING tsend")
        if self.DEBUG: print ("DEBUG 248: Retrans: ", stats_retrans)

        # KN: Enabling garbage collection
        gc.enable()
        gc.collect()
        payload = ""
        blocktbs = []
        payload  = []
        packet = ""
        return rcvr_addr, stats_psent, stats_retrans, FAILED, saved_size, difference_t0


    def _crecv(self, the_sock, my_addr, snd_addr):

        # Shortening addresses to last 8 bytes
        if self.DEBUG: print("DEBUG 264: my_addr, snd_addr: ", my_addr, snd_addr)

        # Buffer storing the received data to be returned
        rcvd_data = b''
        last_check = 0

        next_acknum = self.ONE
        SENDER_ADDR_KNOWN = True
        if (snd_addr == self.ANY_ADDR) or (snd_addr == b''): SENDER_ADDR_KNOWN = False
        self.p_resend = 0  ###

        # Enabling garbage collection
        gc.enable()
        gc.collect()
        while True:
            try:
                the_sock.setblocking(True)
                packet = the_sock.recv(230)
                if self.DEBUG: print("DEBUG 283: packet received: ", packet)
                inp_src_addr, inp_dst_addr, inp_seqnum, inp_acknum, is_ack, last_pkt, check, content = self.__unpack(
                    packet)
                # getting sender address, if unknown, with the first packet
                if (not SENDER_ADDR_KNOWN):
                    snd_addr = inp_src_addr
                    SENDER_ID_KNOWN = True
                # Checking if a "valid" packet... i.e., either for me or broadcast
                if (inp_dst_addr != my_addr) and (inp_dst_addr != self.ANY_ADDR):
                    print("DISCARDED received packet not for me!!")
                    continue
            except (BOARD.LoRaTimeoutError, socket.timeout):
                print("EXCEPTION!! Socket timeout: ", time.time())
                continue
            except Exception as e:
                print("EXCEPTION!! Packet not valid: ", e)
                continue

            print("DEBUG 300: get_checksum(content)", self.__get_checksum(content))
            checksum_OK = (check == self.__get_checksum(content))
            print(check)
            print(snd_addr == inp_src_addr)
            print((checksum_OK))
            print(next_acknum == inp_acknum)
            if (checksum_OK) and (next_acknum == inp_acknum) and (snd_addr == inp_src_addr):
                rcvd_data = content
                last_check = check

                # Sending ACK
                next_acknum = (inp_acknum + self.ONE) % 2
                ack_segment = self.__make_pkt(my_addr, inp_src_addr, inp_seqnum, next_acknum, self.ITS_ACK_PKT,
                                                 last_pkt, b'')
                if self.DEBUG: print("DEBUG 310: Forwarded package", self.p_resend)  ###
                self.p_resend = self.p_resend + 1  ###
                the_sock.setblocking(False)
                time.sleep(0.1)
                s = the_sock.send(ack_segment)
                if self.DEBUG: print("DEBUG 314: Sent ACK", ack_segment)
                if (last_pkt):
                    break
            elif (checksum_OK) and (last_check == check) and (snd_addr == inp_src_addr):
                # KN: Handlig ACK lost
                rcvd_data = content

                # KN: Re-Sending ACK
                next_acknum = (inp_acknum + self.ONE) % 2
                ack_segment = self.__make_pkt(my_addr, inp_src_addr, inp_seqnum, next_acknum, self.ITS_ACK_PKT,
                                                 last_pkt, b'')
                if self.DEBUG: print("DEBUG 325: Forwarded package", self.p_resend)  ###
                the_sock.setblocking(False)
                time.sleep(0.1)
                the_sock.send(ack_segment)
                print("DEBUG 328: re-sending ACK", ack_segment)
                if (last_pkt):
                    break
            else:
                print("DEBUG 332: packet not valid", packet)

        # KN: Enabling garbage collection
        last_check = 0
        packet = ""
        content = ""
        gc.enable()
        gc.collect()
        return rcvd_data, snd_addr

    def connect(self, dest=ANY_ADDR):
        print("loractp: conectandose a... ", dest)
        rcvr_addr, stats_psent, stats_retrans, FAILED = self._csend(b'CONNECT', self.lora, self.lora_mac, dest)
        return self.lora_mac, rcvr_addr, stats_retrans, FAILED

    def listen(self, sender=ANY_ADDR):
        print("loractp: esperando conexiones para...", sender)
        rcvd_data, snd_addr = self._crecv(the_sock=self.lora, my_addr=self.lora_mac, snd_addr=sender)
        if rcvd_data == b"CONNECT":
            print('Conexión establecida!!!')
            return self.lora_mac, snd_addr, 0
        else:
            return self.lora_mac, snd_addr, -1

    def sendit(self, addr=ANY_ADDR, payload=b''):
        rcvr_addr, stats_psent, stats_retrans, FAILED, saved_size, difference_t0 = self._csend(payload, self.lora, self.lora_mac, addr)
        return rcvr_addr,stats_psent, stats_retrans, FAILED, saved_size, difference_t0

    def recvit(self, addr=ANY_ADDR):
        rcvd_data, snd_addr = self._crecv(self.lora, self.lora_mac, addr)
        return rcvd_data, snd_addr
