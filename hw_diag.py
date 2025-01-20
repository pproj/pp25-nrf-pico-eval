import utime
from machine import Pin
from micropython import const

from feedback import FeederBacker


def interrupt_test(irq_pin: int, feeder_backer: FeederBacker):
    p = Pin(irq_pin, pull=Pin.PULL_UP, mode=Pin.IN)

    flag = False

    def handler(_):
        nonlocal flag
        nonlocal feeder_backer
        feeder_backer.led_builtin.on()
        flag = True

    p.irq(handler, trigger=Pin.IRQ_FALLING)

    try:
        print("irq for falling edge on", p)
        cnt = 0
        while True:
            utime.sleep_ms(const(10))
            if flag:
                feeder_backer.led1.on()
                print(f"\033[K\r{cnt} ", end="")
                flag = False
                cnt += 1
                feeder_backer.led_builtin.off()
    finally:
        p.irq(None)


def led_test(feeder_backer: FeederBacker):
    pattern = [
        (True, False, False, False),
        (False, False, False, True),
        (True, False, True, False),
        (False, True, False, False)
    ]

    print("blinking leds...")

    while True:
        for scene in pattern:
            feeder_backer.scene(*scene)
            utime.sleep_ms(250)
