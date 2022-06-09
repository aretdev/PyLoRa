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

from SX127x import board_config, constants, LoRa

board_config.BOARD.setup()


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

        self.lora = LoRa.LoRa(verbose=False,
                              do_calibration=True,
                              calibration_freq=868,
                              sf=7,
                              cr=constants.CODING_RATE.CR4_5,
                              freq=869)

        self.lora.set_mode(constants.MODE.STDBY)
        self.lora.set_pa_config(pa_select=1)

        self.lora_mac = binascii.hexlify(bytes(getmac.get_mac_address(), encoding='utf8'))
        self.my_addr = self.lora_mac[8:]

    def __make_pkt(self, source, destination, seqnum, acknum, pkt_type, is_last, payload):

        print(payload)
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
            print(check)
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

        sender_addr, destination_addr, flags, check = struct.unpack(self.HEADER_FORMAT, header)
        seqnum = self.ONE if (flags & 1) & 1 else self.ZERO
        acknum = self.ONE if (flags >> 2) & 1 else self.ZERO
        is_last = (flags >> 4) & 1 == 1
        pkt_type = (flags >> 6) & 1 == 1
        if content == b'':
            payload = b''
        else:
            payload = content

        return sender_addr, destination_addr, seqnum, acknum, pkt_type, is_last, check, payload

    def __get_checksum(self, data):

        if self.DEBUG: print("DEBUG >> Data Before Getting Checksum: ", data)
        h = hashlib.sha256(data)
        ha = binascii.hexlify(h.digest())

        if self.DEBUG: print("DEBUG >> Checksum: ", ha[-3:])

        return (ha[-3:])

    def _csend(self, content, lora_obj, sender_addr, receiver_addr):
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
        lora_obj.set_timeout(5)
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
                    if self.DEBUG: print("DEBUG >> Waiting for ack...")
                    lora_obj.set_timeout(timeout_value)
                    lora_obj.recv()
                    recv_time = time.time()
                    if self.DEBUG: print("DEBUG >> Ack received!")
                    ack = lora_obj.payload
                    print("p2->  ", lora_obj.payload)
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

                if self.DEBUG: print("DEBUG >> TRYING ATTEMPT:  ", 3 - keep_trying)

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

    def _crecv(self, lora_obj, my_addr, send_addr):
        my_addr = my_addr[8:]
        snd_addr = send_addr[8:]
        if self.DEBUG: print("DEBUG >> my_addr, snd_addr: ", my_addr, snd_addr)
        if self.DEBUG: print("DEBUG >> my_addr, snd_addr: ", my_addr, snd_addr)

        # Buffer storing the received data to be returned
        rcvd_data = b''
        last_check = 0

        next_acknum = self.ONE
        lora_obj.set_timeout(5)
        if (snd_addr == self.ANY_ADDR) or (snd_addr == b''): SENDER_ADDR_KNOWN = False
        self.p_resend = 0  ###

        # Enabling garbage collection
        gc.enable()
        gc.collect()

        while True:
            try:
                lora_obj.recv()
                packet = bytes(lora_obj.payload, encoding="utf-8")
                if self.DEBUG: print("DEBUG >> packet received: ", packet)
                inp_src_addr, inp_dst_addr, inp_seqnum, inp_acknum, is_ack, last_pkt, check, content = self.__unpack(
                    packet)

                # getting sender address, if unknown, with the first packet
                if not SENDER_ADDR_KNOWN:
                    snd_addr = inp_src_addr
                    SENDER_ID_KNOWN = True
                # Checking if a "valid" packet... i.e., either for me or broadcast
                if (inp_dst_addr != my_addr) and (inp_dst_addr != self.ANY_ADDR):
                    if self.DEBUG: print("DEBUG >> DISCARDED received packet not for me!!")
                    continue
            except socket.timeout:
                if self.DEBUG: print("EXCEPTION!! Socket timeout: ", time.time())
                continue
            except Exception as e:
                print("EXCEPTION!! Packet not valid: ", e)
                continue

            if self.DEBUG: print("DEBUG >> get_checksum(content)", self.__get_checksum(content))
            checksum_OK = (check == self.__get_checksum(content))

            if checksum_OK and (next_acknum == inp_acknum) and (snd_addr == inp_src_addr):
                rcvd_data += content
                last_check = check

                # Sending ACK
                next_acknum = (inp_acknum + self.ONE) % 2
                ack_segment = self.__make_pkt(my_addr, inp_src_addr, inp_seqnum, next_acknum, self.ITS_ACK_PKT,
                                              last_pkt, b'')
                if self.DEBUG: print("DEBUG >> Forwarded package", self.p_resend)  ###
                self.p_resend = self.p_resend + 1  ###
                lora_obj.send(ack_segment)
                if self.DEBUG: print("DEBUG >> Sent ACK", ack_segment)
                if last_pkt:
                    break
            elif checksum_OK and (last_check == check) and (snd_addr == inp_src_addr):
                # KN: Handlig ACK lost
                rcvd_data += content

                # KN: Re-Sending ACK
                next_acknum = (inp_acknum + self.ZERO) % 2
                ack_segment = self.__make_pkt(my_addr, inp_src_addr, inp_seqnum, next_acknum, self.ITS_ACK_PKT,
                                              last_pkt, b'')
                self.conta = self.conta - 1
                if self.DEBUG: print("DEBUG >> Forwarded package", self.p_resend)  ###
                lora_obj.send(ack_segment)
                if self.DEBUG: print("DEBUG >> re-sending ACK", ack_segment)
                if last_pkt:
                    break
            else:
                if self.DEBUG: print("DEBUG >> packet not valid", packet)

            # KN: Enabling garbage collection
        last_check = 0
        packet = ""
        content = ""
        gc.enable()
        gc.collect()
        return rcvd_data, snd_addr

    def connect(self, dest=ANY_ADDR):
        print("loractp: connecting to... ", dest)
        rcvr_addr, stats_psent, stats_retrans, FAILED = self._csend(b'CONNECT', self.lora, self.my_addr, dest)
        return self.my_addr, rcvr_addr, stats_retrans, FAILED

    def listen(self, sender=ANY_ADDR):
        print("loractp: listening for...", sender)
        rcvd_data, snd_addr = self._crecv(self.lora, self.lora_mac, sender)
        if rcvd_data == b"CONNECT":
            return self.my_addr, snd_addr, 0
        else:
            return self.my_addr, snd_addr, -1

    def sendit(self, addr=ANY_ADDR, payload=b''):
        rcvr_addr, stats_psent, stats_retrans, FAILED = self._csend(payload, self.lora, self.my_addr, self.ANY_ADDR)
        return rcvr_addr, stats_retrans, FAILED

    def recvit(self, addr=ANY_ADDR):
        rcvd_data, snd_addr = self._crecv(self.lora, self.lora_mac, addr)
        return rcvd_data, snd_addr

    def test(self):
        s = self.__make_pkt(self.my_addr, self.ANY_ADDR, self.ZERO, self.ONE, self.ITS_DATA_PKT, True, b'CONNECT')
        self.lora.send(s)
        self.lora.set_timeout(5)
        self.lora.recv()

