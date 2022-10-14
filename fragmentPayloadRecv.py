
from loractp import CTPLoraEndPoint

ctcp = CTPLoraEndPoint()

try:
    while 1:
       x = ctcp.recvit()
except KeyboardInterrupt:
    print("Cerrando pyLora...")
