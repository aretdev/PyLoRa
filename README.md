
# PyLora

In this work, we look for a development and ampliation of a protocol oriented to reliable communications, based of LoRa technology, this technology is characterized by low consumption, although with low transfer rates and small packet sizes. Therefore, the transmission of larger data results in an unreliable communication since it is not natively supported, it is therefore necessary to develop specific protocols for this matter. Originally the protocol to be develop was implemented on the PyCom platform, it is sought to extend it to new devices.

This library is based on https://github.com/mayeranalytics/pySX127x.




## Usage/Examples

Sender Exaple:
```Python
from pyLora import pyLora

try:

    pyLora = pyLora(verbose=True, freq=867)
    pyLora.send(b'Hello world!')
except KeyboardInterrupt:
    print("Closing pyLora...")

```

Receiver Example:
```Python
from pyLora import pyLora

try:
    pyLora = pyLora(verbose=True, freq=867)
    pyLora.setblocking(True)
    payload = pyLora.recv()
except KeyboardInterrupt:
    print("Closing pyLora...")

```
