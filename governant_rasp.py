from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()


class GobernantBerry(LoRa):
    tx_ready = 0
    n_data_recieved = 0
    waiting_rx = False

    def __init__(self, verbose=True, do_calibration=True, calibration_freq=868, sf=7, cr=CODING_RATE.CR4_5, freq=869):
        super(GobernantBerry, self).__init__(verbose, do_calibration, calibration_freq, sf, cr, freq)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([1, 0, 0, 0, 0, 0])  # Initialize DIO0 for TxDone, first I need to send something

    # When I send data is done, change to receive mode
    def on_tx_done(self):
        self.clear_irq_flags(TxDone=1)  # clear txdone IRQ flag

        # I just TX so I prepare for rx
        self.set_dio_mapping([0] * 6)
        self.set_mode(MODE.RXCONT)
        self.tx_ready = 0  # I'm ready to transmit again!

    def on_rx_done(self):
        print("\nI recieved something! :")
        payload = self.read_payload(nocheck=True)
        self.clear_irq_flags(RxDone=1)  # clear rxdone IRQ flag
        self.reset_ptr_rx()  # we clear pointer
        self.set_mode(MODE.RXCONT)
        self.waiting_rx = False

    def start(self):
        sys.stdout.write("\rstart\n")
        while True:
            # Am I ready to tx and im not waiting for rx?

            if not self.tx_ready and not self.waiting_rx:
                # I'm ready to transmit! I go to sleep mode and map my DIOs
                self.set_mode(MODE.SLEEP)
                self.set_dio_mapping([1, 0, 0, 0, 0, 0])  # DIO0 = 1 is for TXDone, transmitting mode basically
                self.set_mode(MODE.STDBY)
                value = input("Please enter a string:\n")

                if value == "quit":
                    break

                formatted = list(value.encode('ascii'))
                self.write_payload(formatted)  # I send my payload to LORA SX1276 interface
                self.set_mode(MODE.TX)  # I enter on TX Mode
                self.waiting_rx = True  # now I start to wait until I get some data!
                self.tx_ready = 1  # Also, im not ready to tx again until the current tx is done


lora = GobernantBerry(verbose=False)
print(lora)

lora.set_pa_config(pa_select=1)
lora.start()
