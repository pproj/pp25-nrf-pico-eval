from machine import Pin
from micropython import const
from const import *


class FeederBacker:

    def __init__(self, led1_pin: int = LED1_PIN, led2_pin: int = LED2_PIN, led3_pin: int = LED3_PIN):
        self._led_builtin = Pin(LED_BUILTIN_PIN, Pin.OUT)
        self._led1 = Pin(led1_pin, Pin.OUT)
        self._led2 = Pin(led2_pin, Pin.OUT)
        self._led3 = Pin(led3_pin, Pin.OUT)
        self.reset()

    def reset(self):
        self._led_builtin.off()
        self._led1.off()
        self._led2.off()
        self._led3.off()

    def scene(self, led_builtin: bool = None, led1: bool = None, led2: bool = None, led3: bool = None):
        if led_builtin is not None:
            self._led_builtin(led_builtin)

        if led1 is not None:
            self._led1(led1)

        if led2 is not None:
            self._led2(led2)

        if led3 is not None:
            self._led3(led3)

    @property
    def led_activity(self):
        # activity led is configurable
        return self._led_builtin

    @property
    def led_builtin(self):
        return self._led_builtin

    @property
    def led1(self) -> Pin:
        return self._led1

    @property
    def led2(self) -> Pin:
        return self._led2

    @property
    def led3(self) -> Pin:
        return self._led3
