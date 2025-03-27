from micropython import const

DEVICE_REVISION = const(3)
LED_BUILTIN_PIN = const(25)

# Proto 3:
if DEVICE_REVISION == 3:
    IRQ_PIN = 20

    MOSI_PIN = 19
    MISO_PIN = 16
    SCK_PIN = 18
    CSN_PIN = 21
    CE_PIN = 17

    SPI_DEV = 0

    LED1_PIN = 27
    LED2_PIN = 26
    LED3_PIN = 22

else:
    IRQ_PIN = 7

    MOSI_PIN = 15
    MISO_PIN = 8
    SCK_PIN = 10
    CSN_PIN = 14
    CE_PIN = 13

    SPI_DEV = 1

    LED1_PIN = 22
    LED2_PIN = 26
    LED3_PIN = 27
