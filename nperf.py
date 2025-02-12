import utime
from machine import Pin
from micropython import const

from const import IRQ_PIN
from feedback import FeederBacker
from nrf24l01 import NRF24L01, RX_DR, TX_DS, MAX_RT, CONFIG, PWR_UP, PRIM_RX

DIRECTION_RX = const(1)
DIRECTION_TX = const(2)

DATA = b"NPERF"


def nperf(nrf: NRF24L01, feedback: FeederBacker, direction: int):
    pkts = 0
    p = Pin(IRQ_PIN, pull=Pin.PULL_UP, mode=Pin.IN)

    last_printout = utime.ticks_ms()

    tx_queue = 0

    if direction == DIRECTION_RX:
        nrf.start_listening()

    if direction == DIRECTION_TX:
        nrf.reg_write(CONFIG, (nrf.reg_read(CONFIG) | PWR_UP) & ~PRIM_RX)  # switch to tx
        nrf.ce(1)  # we will keep this high

    utime.sleep_us(130)

    while True:
        # printout
        dt = utime.ticks_diff(utime.ticks_ms(), last_printout)
        if dt >= 5000:
            pktsps = pkts / (dt / 1000)
            print(f"{pktsps:.2f} pkts/s")
            pkts = 0
            last_printout = utime.ticks_ms()

        # do thing

        if direction == DIRECTION_RX:
            if not p.value():  # irq
                feedback.led_activity.on()
                status = nrf.read_status()
                if status & RX_DR:
                    while nrf.any():
                        _ = nrf.recv()  # clears the flag apparently
                        pkts += 1

                feedback.led_activity.off()

        if direction == DIRECTION_TX:
            if tx_queue < 3:
                nrf.put_tx_buf(DATA)

            if not p.value():  # irq
                status = nrf.read_status()
                if status & TX_DS:
                    feedback.led1.off()
                    nrf.clear_irq()
                    pkts += 1
                    tx_queue -= 1

                if status & MAX_RT:  # send failed...
                    nrf.ce(0)
                    feedback.led1.on()
                    nrf.clear_irq()
                    nrf.flush_tx()
                    tx_queue = 0
                    nrf.ce(1)
                    utime.sleep_us(130)
