# pysx127x

This is a fork of the original PySX127x but adapted to v1.4 instead of 1.0

### Some questions that I had

##### Why everytime we try to write into a reg, we use OR operator along with the address?

  According to SX1272/73 datasheet, it is part of the SPI interface for writing the registers!

  the first byte is the adress Ex. RegOpMode : 0x01

  we want to write into that register so we OR it with 0x80 -> 0x81

  0x80 = 1000 0000 \
  0x01 = 0000 0001 \
  0x81 = 1000 0001 -> we are on write mode!   

  for more information about this chip you can visit : https://www.mouser.com/datasheet/2/761/sx1272-1277619.pdf
