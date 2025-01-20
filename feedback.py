from machine import Pin

LED_BUILTIN_PIN = 25


class FeederBacker:

    def __init__(self, led1_pin: int, led2_pin: int, led3_pin: int):
        self._led_builtin = Pin(LED_BUILTIN_PIN, Pin.OUT)
        self._led1 = Pin(led1_pin, Pin.OUT)
        self._led2 = Pin(led2_pin, Pin.OUT)
        self._led3 = Pin(led3_pin, Pin.OUT)

    def reset(self):
        self._led_builtin.off()
        self._led1.off()
        self._led2.off()
        self._led3.off()

    def scene(self, led_builtin: bool = False, led1: bool = False, led2: bool = False, led3: bool = False):
        self._led_builtin(led_builtin)
        self._led1(led1)
        self._led2(led2)
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
