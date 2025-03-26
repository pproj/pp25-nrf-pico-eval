from micropython import const

# Proto 3:
IRQ_PIN = const(20)

MOSI_PIN = const(19)
MISO_PIN = const(16)
SCK_PIN = const(18)
CSN_PIN = const(21)
CE_PIN = const(17)

SPI_DEV = const(0)

# Proto 1 and 2:
# IRQ_PIN = const(7)
#
# MOSI_PIN = const(15)
# MISO_PIN = const(8)
# SCK_PIN = const(10)
# CSN_PIN = const(14)
# CE_PIN = const(13)
#
# SPI_DEV = const(1)
