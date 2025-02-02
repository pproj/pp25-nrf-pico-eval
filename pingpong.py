import utime

from micropython import const
from feedback import FeederBacker
from nrf24l01 import NRF24L01, MODE_PRIM_TX, CONFIG, PRIM_RX, PWR_UP, RX_DR, TX_DS, MAX_RT
from machine import Pin

STATE_PREPARING_SEND = const(1)
STATE_SEND = const(2)
STATE_SEND_WAIT_COMPLETE = const(3)
STATE_PREPARING_RECV = const(4)
STATE_RECV = const(5)

from const import IRQ_PIN

DATA = b"PINGPONG"


# TODO: For some reason after a while, both parties stuck in sending mode
# Maybe
#  - the ACK packet gets lost?
#  - brown out?
# setting PA to min seems to ease the issue, but does not solve it...
# sometimes they can both stuck in rx state... wtf
# It seems like adding a sleep at a specific part kinda stabilizes it..... WTF?!?!?!

def pingpong(nrf: NRF24L01, feedback: FeederBacker, has_serve: bool):
    # led1: rx
    # led2: tx
    # led3: problem
    print("Playing ping-pong")
    print()
    if has_serve:
        state = STATE_PREPARING_SEND
    else:
        state = STATE_PREPARING_RECV

    p = Pin(IRQ_PIN, pull=Pin.PULL_UP, mode=Pin.IN)

    # power on the nrf module
    nrf.power_up(MODE_PRIM_TX)  # mode doesn't matter, we will change it on the fly

    hits = 0
    last_printout = utime.ticks_ms()

    while True:
        dt = utime.ticks_diff(utime.ticks_ms(), last_printout)
        if dt >= 5000:
            hps = hits / (dt / 1000)
            print(f"{hps:.2f} hits/s")
            hits = 0
            last_printout = utime.ticks_ms()

        # send

        if state == STATE_PREPARING_SEND:
            # indicate send mode
            feedback.led1.off()
            feedback.led2.on()  # tx

            nrf.clear_irq()
            nrf.flush_tx()
            nrf.reg_write(CONFIG, (nrf.reg_read(CONFIG) | PWR_UP) & ~PRIM_RX)  # switch to tx
            utime.sleep_us(130)  # this is probably not needed
            state = STATE_SEND

        if state == STATE_SEND:
            # send the data
            feedback.led_activity.on()
            nrf.put_tx_buf(DATA)

            # enable the chip so it can send the data
            nrf.ce(1)
            utime.sleep_us(15)  # needs to be >10us
            nrf.ce(0)
            feedback.led_activity.off()
            state = STATE_SEND_WAIT_COMPLETE

        if state == STATE_SEND_WAIT_COMPLETE:
            if not p.value():  # irq
                status = nrf.read_status()
                nrf.clear_irq()
                if status & TX_DS:
                    # data sent, we are good
                    feedback.led3.off()
                    state = STATE_PREPARING_RECV

                if status & MAX_RT:
                    # sending failed. we may retry
                    nrf.flush_tx()
                    feedback.led3.on()  # turn on led3 to indicate connection problems
                    state = STATE_SEND

        # recv

        if state == STATE_PREPARING_RECV:
            # indicate recv mode
            feedback.led1.on()  # rx
            feedback.led2.off()

            nrf.clear_irq()
            nrf.flush_rx()
            nrf.reg_write(CONFIG, nrf.reg_read(CONFIG) | PWR_UP | PRIM_RX)  # switch to rx
            nrf.ce(1)  # start listening
            utime.sleep_us(130)  # Tstby2a
            state = STATE_RECV

        if state == STATE_RECV:
            if not p.value():  # irq
                utime.sleep_us(400)  # maybe needs time to send ACK??? IT DOES!!! WTF?!!!!
                status = nrf.read_status()
                if status & RX_DR:
                    nrf.ce(0)  # stop listening
                    data_recv = nrf.recv()  # clears the flag apparently
                    if data_recv[:8] == DATA:
                        hits += 1  # hit the ball back
                        state = STATE_PREPARING_SEND
                    else:
                        print("Ball dropped: Invalid data received")
                        return
