from os import urandom

import time
from micropython import const
import random
import utime

from feedback import FeederBacker

from machine import SPI, Pin

from nrf24l01 import NRF24L01, POWER_0, SPEED_250K, RX_DR

from const import *

ADDRESS = b'\xe1\xf0\xf0\xf0\xf0'
CHANNEL = const(6)

# TODO: News flash! For some reason this is broken now \o/
# It does not work with the non-pa modules


def display_byte(feeder_backer: FeederBacker, data: int):
    feeder_backer.scene(
        led1=bool(data & 0x01),
        led2=bool(data & 0x02),
        led3=bool(data & 0x04)
    )


def demo():
    feeder_backer = FeederBacker()

    # indicate demo mode activation
    for i in range(5):
        feeder_backer.led1.on()
        utime.sleep_ms(100)
        feeder_backer.led1.off()
        utime.sleep_ms(150)

    irq = Pin(IRQ_PIN, mode=Pin.IN, pull=Pin.PULL_UP)
    csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)
    ce = Pin(CE_PIN, mode=Pin.OUT, value=0)
    spidev = SPI(SPI_DEV, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    nrf = NRF24L01(spidev, csn, ce, payload_size=1, channel=CHANNEL)
    nrf.set_power_speed(POWER_0, SPEED_250K)
    nrf.set_crc(2)  # 16 bit
    nrf.open_tx_pipe(ADDRESS)  # using the same for tx and rx
    nrf.open_rx_pipe(0, ADDRESS)
    nrf.set_auto_ack(False)
    nrf.setup_retr(0, 0)  # disable auto retransmission (no_ack would disable it anyway, but whatever)

    wait_max = random.randrange(10000, 30000)  # between 10 and 30 sec
    print(f"wait_max: {wait_max} ms...")

    nrf.start_listening()

    last_msg = utime.ticks_ms()
    while True:
        if utime.ticks_diff(utime.ticks_ms(), last_msg) > wait_max:
            break  # become boss

        if not irq.value():  # low active
            feeder_backer.led_activity.on()
            status = nrf.read_status()
            if status & RX_DR:
                nrf.clear_irq()

            while nrf.any():
                meme = nrf.recv()
                display_byte(feeder_backer, meme[0])

            print(".", end="")
            last_msg = utime.ticks_ms()
            feeder_backer.led_activity.off()

    nrf.stop_listening()

    print("becoming boss")

    # boss mode
    while True:
        time.sleep_ms(250)
        feeder_backer.led_activity.on()
        meme = urandom(1)
        nrf.send_no_ack(meme)
        display_byte(feeder_backer, meme[0])
        print("!", end="")
        feeder_backer.led_activity.off()
