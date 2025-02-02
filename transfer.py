from machine import Pin
from micropython import const
import utime

from feedback import FeederBacker
from nrf24l01 import NRF24L01, RX_DR, CONFIG, PWR_UP

IRQ_PIN = const(7)


def rx_poll(nrf: NRF24L01, feeder_backer: FeederBacker):
    nrf.start_listening()

    while True:
        if nrf.any():
            while nrf.any():
                feeder_backer.led_activity.on()
                buf = nrf.recv()
                yield buf

                utime.sleep_ms(const(15))  # why is this here?
                feeder_backer.led_activity.off()

            # Give master time to get into receive mode.
            utime.sleep_ms(const(10))


def rx_irq(nrf: NRF24L01, feeder_backer: FeederBacker):
    nrf.start_listening()

    events = 0

    def handler(_):
        nonlocal events
        nonlocal feeder_backer
        feeder_backer.led_activity.on()
        events += 1

    p = Pin(IRQ_PIN, pull=Pin.PULL_UP, mode=Pin.IN)
    p.irq(handler, trigger=Pin.IRQ_FALLING)

    try:
        while True:
            if events == 0:
                continue

            events -= 1

            status = nrf.read_status()
            if status & RX_DR:
                nrf.clear_irq()

                while nrf.any():  # maybe more than one message stuck in the fifo
                    buf = nrf.recv()
                    yield buf

                utime.sleep_ms(const(15))  # why is this here?

            feeder_backer.led_activity.off()
    finally:
        p.irq(None)


def tx_poll(nrf: NRF24L01, feeder_backer: FeederBacker, generator: callable, callback: callable = None):
    nrf.stop_listening()

    for msg in generator():
        feeder_backer.led_activity.on()

        success = True
        err_msg = ""
        try:
            nrf.send(msg, timeout=50)
        except OSError as e:
            success = False
            err_msg = str(e)

        stop = False  # will be none by default
        if callback:
            stop = callback(success, err_msg)

        feeder_backer.led_activity.off()

        if stop:
            break


def tx_irq(nrf: NRF24L01, feeder_backer: FeederBacker, generator: callable, callback: callable = None):
    nrf.stop_listening()

    flag = False

    def handler(_):
        nonlocal flag
        flag = True

    p = Pin(IRQ_PIN, pull=Pin.PULL_UP, mode=Pin.IN)
    p.irq(handler, trigger=Pin.IRQ_FALLING)

    try:
        for msg in generator():
            feeder_backer.led_activity.on()

            nrf.send_start(msg)  # powers up

            start = utime.ticks_ms()
            timed_out = False
            while (not flag) and (not timed_out):
                # wait for interrupt
                timed_out = utime.ticks_diff(utime.ticks_ms(), start) > 100  # time out after 100ms

            success = True
            err_msg = ""

            if timed_out:  # timed out
                nrf.flush_tx()  # cancel
                nrf.reg_write(CONFIG, nrf.reg_read(CONFIG) & ~PWR_UP)  # power down

                success = False
                err_msg = "timed out"

            elif flag:  # hw signaled... something
                flag = False
                result = nrf.send_done()

                if result == 2:
                    nrf.flush_tx()  # cancel
                    success = False
                    err_msg = "send failed"
            else:
                raise Exception("something wrong")

            stop = False  # will be none by default
            if callback:
                stop = callback(success, err_msg)

            feeder_backer.led_activity.off()

            if stop:
                break

    finally:
        p.irq(None)  # disable irq
