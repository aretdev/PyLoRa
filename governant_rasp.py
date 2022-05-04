import sys
from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

class GobernantBerry(LoRa):
    n_data_recieved = 0

    def __init__(self, verbose=True, do_calibration=True, calibration_freq=868, sf=8, cr=CODING_RATE.CR4_5, freq=868):
        super(GobernantBerry, self).__init__(verbose, do_calibration, calibration_freq, sf, cr, freq)
        self.set_mode(MODE.SLEEP)
        # revisar los dio mappings
        
    def on_tx_done(self):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        sys.stdout.flush()
        sys.stdout.write("\rtx #%d")
        sleep(1)
        self.write_payload([0x0f])
        self.set_mode(MODE.TX)

    def start(self):
        sys.stdout.write("\rstart")
        self.write_payload([0x0f])
        self.set_mode(MODE.TX)
        while True:
            print("waiting, going to sleep 1")
            sleep(1)


lora = GobernantBerry(verbose=False)
print(lora)

lora.set_pa_config(pa_select=1)

lora.start()
