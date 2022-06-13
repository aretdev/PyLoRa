
from loractp import CTPLoraEndPoint

ctp = CTPLoraEndPoint(DEBUG=True)

ctp.connect()

import gc
import loractp





# gc.enable()
#
# ctpc = loractp.CTPLoraEndPoint()
#
# myaddr, rcvraddr, status = ctpc.listen()
# if status == 0:
#     print("pong.py: connection from {} to me ({})".format(rcvraddr, myaddr))
# else:
#     print("pong.py: failed connection from {} to me ({})".format(rcvraddr, myaddr))
#
# while True:
#
#     print('pong.py: waiting for data from ', rcvraddr)
#     try:
#         rcvd_data, addr = ctpc.recvit(rcvraddr)
#         print("pong.py: got ", rcvd_data, addr)
#     except Exception as e:
#         print("pong.py: EXCEPTION!! ", e)
#         break
#
#     tbs = '{"type": "PONG", "value": rcvd_data, "time": 69}'
#     tbsb = str.encode(tbs)
#     print('--->pong.py: sending ', tbsb)
#     try:
#         addr, quality, result = ctpc.sendit(rcvraddr, tbsb)
#         print("pong.py: ACK from {} (quality = {}, result = {})".format(addr, quality, result))
#     except Exception as e:
#         print("pong.py: EXCEPTION!! ", e)
#         break
#
#     tbs = ""
#     tbsj = ""
#     tbsb = ""
#     gc.collect()
