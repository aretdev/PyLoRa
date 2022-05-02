""" Defines the BOARD class that contains the board pin mappings and RF module HF/LF info. """
# -*- coding: utf-8 -*-

# Copyright 2015-2018 Mayer Analytics Ltd.
#
# This file is part of pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.


import RPi.GPIO as GPIO
import spidev

import time


class BOARD:
    """ Board initialisation/teardown and pin configuration is kept here.
        Also, information about the RF module is kept here.
        Pin mapped for Rpi with dragino Hat Lora/GPS v1.4
        Fo reference of mapping see : https://pinout.xyz/pinout/pin7_gpio4
        with :
            https://github.com/dragino/Lora/blob/master/Lora_GPS%20HAT/v1.4/
    """
    # Note that the BCOM numbering for the GPIOs is used.
    DIO0 = 4        # RaspPi GPIO 4
    DIO1 = None     # Not necessary, we could still using them if we wanted to
    DIO2 = None     # Not necessary, we could still using them if we wanted to
    DIO3 = None     # Not necessary, we could still using them if we wanted to
    NSS = 25        # RasPi GPIO25, connection required!
    RST = 17        # Raspi GPIO17, this is not really required
    LED = None      # Dragino Raspberry PI hat (no onboard led)
    SWITCH = None   # No switch

    # The spi object is kept here
    spi = None
    SPI_BUS = 0
    SPI_CS = 0

    # tell pySX127x here whether the attached RF module uses low-band
    # (RF*_LF pins) or high-band (RF*_HF pins).
    # low band (called band 1&2) are 137-175 and 410-525
    # high band (called band 3) is 862-1020
    low_band = True

    @staticmethod
    def setup():
        """ Configure the Raspberry GPIOs
        :rtype : None
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # LED, RST & NSS
        if BOARD.LED is not None:
            GPIO.setup(BOARD.LED, GPIO.OUT)

        GPIO.setup(BOARD.RST, GPIO.OUT)
        GPIO.setup(BOARD.NSS, GPIO.OUT)
        if BOARD.LED is not None:
            GPIO.output(BOARD.LED, 0)

        GPIO.output(BOARD.RST, 0)
        time.sleep(.01)
        GPIO.output(BOARD.RST, 1)
        time.sleep(.01)
        GPIO.output(BOARD.NSS, 1)
        # switch
        if BOARD.SWITCH is not None:
            GPIO.setup(BOARD.SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # DIOx
        for gpio_pin in [BOARD.DIO0, BOARD.DIO1, BOARD.DIO2, BOARD.DIO3]:
            if gpio_pin is not None:
                GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    @staticmethod
    def teardown():
        """ Cleanup GPIO and SpiDev """
        GPIO.cleanup()
        BOARD.spi.close()

    @staticmethod
    def SpiDev():
        """ Init and return the SpiDev object
        :return: SpiDev object
        :param spi_bus: The RPi SPI bus to use: 0 or 1
        :param spi_cs: The RPi SPI chip select to use: 0 or 1
        :rtype: SpiDev
        """
        spi_bus = BOARD.SPI_BUS
        spi_cs = BOARD.SPI_CS
        BOARD.spi = spidev.SpiDev()
        BOARD.spi.open(spi_bus, spi_cs)
        # SX127x can go up to 10MHz, pick half that to be safe
        BOARD.spi.max_speed_hz = 5000000
        return BOARD.spi

    @staticmethod
    def add_event_detect(dio_number, callback):
        """ Wraps around the GPIO.add_event_detect function
        :param dio_number: DIO pin 0...5
        :param callback: The function to call when the DIO triggers an IRQ.
        :return: None
        """
        GPIO.add_event_detect(dio_number, GPIO.RISING, callback=callback)

    @staticmethod
    def add_events(cb_dio0, cb_dio1, cb_dio2, cb_dio3, cb_dio4, cb_dio5, switch_cb=None):
        if BOARD.DIO0 is not None:
            BOARD.add_event_detect(BOARD.DIO0, callback=cb_dio0)
        if BOARD.DIO1 is not None:
            BOARD.add_event_detect(BOARD.DIO1, callback=cb_dio1)
        if BOARD.DIO2 is not None:
            BOARD.add_event_detect(BOARD.DIO2, callback=cb_dio2)
        if BOARD.DIO3 is not None:
            BOARD.add_event_detect(BOARD.DIO3, callback=cb_dio3)
        # the modtronix inAir9B does not expose DIO4 and DIO5
        if (switch_cb is not None) and (BOARD.SWITCH is not None):
            GPIO.add_event_detect(BOARD.SWITCH, GPIO.RISING, callback=switch_cb, bouncetime=300)

    @staticmethod
    def led_on(value=1):
        """ Switch the proto shields LED
        :param value: 0/1 for off/on. Default is 1.
        :return: value
        :rtype : int
        """
        if BOARD.LED is not None:
            GPIO.output(BOARD.LED, value)
        else:
            raise RuntimeError("LED is set no None")
            value = -1
        return value

    @staticmethod
    def led_off(value=0):
        """ Switch LED off
        :return: 0
        """
        if BOARD.LED is not None:
            GPIO.output(BOARD.LED, value)
        else:
            raise RuntimeError("LED is set no None")
            value = -1
        return value

    @staticmethod
    def reset():
        """ manual reset
        :return: 0
        """
        GPIO.output(BOARD.RST, 0)
        time.sleep(.01)
        GPIO.output(BOARD.RST, 1)
        time.sleep(.01)
        return 0

    @staticmethod
    def chip_select(value):
        """ Enable/Disable Chip Select via NSS pin
        :return: value
        """
        if BOARD.NSS is not None:
            GPIO.output(BOARD.NSS, not(value))
        return value

    @staticmethod
    def blink(time_sec, n_blink):
        if BOARD.LED is not None:
            if n_blink == 0:
                return
            BOARD.led_on()
            for i in range(n_blink):
                time.sleep(time_sec)
                BOARD.led_off()
                time.sleep(time_sec)
                BOARD.led_on()
            BOARD.led_off()
        else:
            print("LED is set no None")
