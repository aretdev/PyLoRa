import time

from loractp import CTPLoraEndPoint

ctp = CTPLoraEndPoint(DEBUG=True)

ctp.sendit("e")