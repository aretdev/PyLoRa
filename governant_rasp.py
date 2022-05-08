import sys
from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

class GobernantBerry(LoRa):
    tx_ready = 0;
    n_data_recieved = 0

    def __init__(self, verbose=True, do_calibration=True, calibration_freq=868, sf=7, cr=CODING_RATE.CR4_5, freq=869):
        super(GobernantBerry, self).__init__(verbose, do_calibration, calibration_freq, sf, cr, freq)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([1,0,0,0,0,0])


    def on_tx_done(self):
        self.clear_irq_flags(TxDone=1) # clear txdone IRQ flag
        self.tx_ready = 0;

    def start(self):
        sys.stdout.write("\rstart")
        while True:
            if not self.tx_ready:
                self.set_mode(MODE.STDBY)
                value = input("Please enter a string:\n")

                if value == "quit":
                    break;

                formatted = list(value.encode('ascii'))
                self.write_payload(formatted)
                self.set_mode(MODE.TX)
                self.tx_ready = 1


lora = GobernantBerry(verbose=False)
print(lora)
lora.start()
