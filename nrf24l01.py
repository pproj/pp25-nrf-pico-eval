"""NRF24L01 driver for MicroPython
"""

from micropython import const
import utime

# nRF24L01+ registers
CONFIG = const(0x00)
EN_RXADDR = const(0x02)
SETUP_AW = const(0x03)
RF_CH = const(0x05)
RF_SETUP = const(0x06)
STATUS = const(0x07)
OBSERVE_TX = const(0x08)
RPD = const(0x09)
RX_ADDR_P0 = const(0x0A)
TX_ADDR = const(0x10)
RX_PW_P0 = const(0x11)
FIFO_STATUS = const(0x17)
DYNPD = const(0x1C)

# SETUP_RETR register
SETUP_RETR = const(0x04)
ARD_250US = const(0x00)
ARD_500US = const(0x10)
ARD_750US = const(0x20)
ARD_1000US = const(0x30)
ARD_1250US = const(0x40)
ARD_1500US = const(0x50)
ARD_1750US = const(0x60)
ARD_2000US = const(0x70)
ARD_2250US = const(0x80)
ARD_2500US = const(0x90)
ARD_2750US = const(0xa0)
ARD_3000US = const(0xb0)
ARD_3250US = const(0xc0)
ARD_3500US = const(0xd0)
ARD_3750US = const(0xe0)
ARD_4000US = const(0xf0)

# CONFIG register
EN_CRC = const(0x08)  # enable CRC
CRCO = const(0x04)  # CRC encoding scheme; 0=1 byte, 1=2 bytes
PWR_UP = const(0x02)  # 1=power up, 0=power down
PRIM_RX = const(0x01)  # RX/TX control; 0=PTX, 1=PRX

# RF_SETUP register
POWER_0 = const(0x00)  # -18 dBm
POWER_1 = const(0x02)  # -12 dBm
POWER_2 = const(0x04)  # -6 dBm
POWER_3 = const(0x06)  # 0 dBm
SPEED_1M = const(0x00)
SPEED_2M = const(0x08)
SPEED_250K = const(0x20)
CONT_WAVE = const(0x80)
PLL_LOCK = const(0x10)

# STATUS register
RX_DR = const(0x40)  # RX data ready; write 1 to clear
TX_DS = const(0x20)  # TX data sent; write 1 to clear
MAX_RT = const(0x10)  # max retransmits reached; write 1 to clear

# FIFO_STATUS register
RX_EMPTY = const(0x01)  # 1 if RX FIFO is empty

# EN_AA register
EN_AA = const(0x01)
ENAA_P5 = const(0x20)
ENAA_P4 = const(0x10)
ENAA_P3 = const(0x08)
ENAA_P2 = const(0x04)
ENAA_P1 = const(0x02)
ENAA_P0 = const(0x01)

# FEATURE register
FEATURE = const(0x1d)
EN_DPL = const(0x04)
EN_ACK_PAY = const(0x02)
EN_DYN_ACK = const(0x01)

# constants for instructions
R_RX_PL_WID = const(0x60)  # read RX payload width
R_RX_PAYLOAD = const(0x61)  # read RX payload
W_TX_PAYLOAD = const(0xA0)  # write TX payload
W_TX_PAYLOAD_NOACK = const(0xB0)  # write TX payload
FLUSH_TX = const(0xE1)  # flush TX FIFO
FLUSH_RX = const(0xE2)  # flush RX FIFO
NOP = const(0xFF)  # use to read STATUS register

MODE_PRIM_TX = const(1)
MODE_PRIM_RX = const(2)


class NRF24L01:
    def __init__(self, spi, cs, ce, channel=46, payload_size=16):
        assert payload_size <= 32

        self.buf = bytearray(1)

        # store the pins
        self.spi = spi
        self.cs = cs
        self.ce = ce

        # init the SPI bus and pins
        self._init_spi(4000000)

        # reset everything
        ce.init(ce.OUT, value=0)
        cs.init(cs.OUT, value=1)

        self.payload_size = payload_size
        self.pipe0_read_addr = None
        utime.sleep_ms(5)

        # set address width to 5 bytes and check for device present
        self.reg_write(SETUP_AW, 0b11)
        if self.reg_read(SETUP_AW) != 0b11:
            raise OSError("nRF24L01+ Hardware not responding")

        # disable dynamic payloads
        self.reg_write(DYNPD, 0)

        # auto retransmit delay: 1750us
        # auto retransmit count: 8
        self.setup_retr(ARD_1750US, 8)

        # set rf power and speed
        self.set_power_speed(POWER_3, SPEED_250K)  # Best for point to point links

        # init CRC
        self.set_crc(2)

        # clear status flags
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

        # set channel
        self.set_channel(channel)

        # flush buffers
        self.flush_rx()
        self.flush_tx()

        # enable aa
        self.set_auto_ack(True)

    def _init_spi(self, baudrate: int):
        try:
            master = self.spi.MASTER
        except AttributeError:
            self.spi.init(baudrate=baudrate, polarity=0, phase=0)
        else:
            self.spi.init(master, baudrate=baudrate, polarity=0, phase=0)

    def setup_retr(self, delay: int, count: int):
        assert 15 >= count >= 0
        assert ARD_4000US >= delay >= ARD_250US
        self.reg_write(SETUP_RETR, delay | count)

    def reg_read(self, reg):
        self.cs(0)
        self.spi.readinto(self.buf, reg)
        self.spi.readinto(self.buf)
        self.cs(1)
        return self.buf[0]

    def reg_write_bytes(self, reg, buf):
        self.cs(0)
        self.spi.readinto(self.buf, 0x20 | reg)
        self.spi.write(buf)
        self.cs(1)
        return self.buf[0]

    def reg_write(self, reg, value):
        self.cs(0)
        self.spi.readinto(self.buf, 0x20 | reg)
        ret = self.buf[0]
        self.spi.readinto(self.buf, value)
        self.cs(1)
        return ret

    def read_status(self):
        self.cs(0)
        # STATUS register is always shifted during command transmit
        self.spi.readinto(self.buf, NOP)
        self.cs(1)
        return self.buf[0]

    def observe_tx(self) -> tuple[int, int]:
        # TODO: this does not work...
        val = self.reg_read(OBSERVE_TX)
        plos_cnt = (val & 0xf8) >> 4
        arc_cnt = val & 0x07
        return plos_cnt, arc_cnt

    def flush_rx(self):
        self.cs(0)
        self.spi.readinto(self.buf, FLUSH_RX)
        self.cs(1)

    def flush_tx(self):
        self.cs(0)
        self.spi.readinto(self.buf, FLUSH_TX)
        self.cs(1)

    def clear_irq(self):
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

    # power is one of POWER_x defines; speed is one of SPEED_x defines
    def set_power_speed(self, power, speed):
        setup = self.reg_read(RF_SETUP) & 0b11010001
        self.reg_write(RF_SETUP, setup | power | speed)

    # length in bytes: 0, 1 or 2
    def set_crc(self, length):
        config = self.reg_read(CONFIG) & ~(CRCO | EN_CRC)
        if length == 0:
            pass
        elif length == 1:
            config |= EN_CRC
        else:
            config |= EN_CRC | CRCO
        self.reg_write(CONFIG, config)

    def set_channel(self, channel):
        self.reg_write(RF_CH, min(channel, 125))

    def set_auto_ack(self, auto_ack: bool):
        if auto_ack:
            self.reg_write(EN_AA, ENAA_P0 | ENAA_P1 | ENAA_P2 | ENAA_P3 | ENAA_P4 | ENAA_P5)  # enable for all pipes
        else:
            self.reg_write(EN_AA, 0x00)

    # address should be a bytes object 5 bytes long
    def open_tx_pipe(self, address):
        assert len(address) == 5
        self.reg_write_bytes(RX_ADDR_P0, address)
        self.reg_write_bytes(TX_ADDR, address)
        self.reg_write(RX_PW_P0, self.payload_size)

    # address should be a bytes object 5 bytes long
    # pipe 0 and 1 have 5 byte address
    # pipes 2-5 use same 4 most-significant bytes as pipe 1, plus 1 extra byte
    def open_rx_pipe(self, pipe_id, address):
        assert len(address) == 5
        assert 0 <= pipe_id <= 5
        if pipe_id == 0:
            self.pipe0_read_addr = address
        if pipe_id < 2:
            self.reg_write_bytes(RX_ADDR_P0 + pipe_id, address)
        else:
            self.reg_write(RX_ADDR_P0 + pipe_id, address[0])
        self.reg_write(RX_PW_P0 + pipe_id, self.payload_size)
        self.reg_write(EN_RXADDR, self.reg_read(EN_RXADDR) | (1 << pipe_id))

    def power_up(self, mode: int):
        if mode == MODE_PRIM_RX:
            self.reg_write(CONFIG, self.reg_read(CONFIG) | PWR_UP | PRIM_RX)
        elif mode == MODE_PRIM_TX:
            self.reg_write(CONFIG, (self.reg_read(CONFIG) | PWR_UP) & ~PRIM_RX)
        else:
            raise Exception("nemjolet")
        utime.sleep_us(2000)  # needs to be at least 1.5ms ... for some reason 1.5ms fails for small payloads ... weird

    def start_listening(self):
        self.power_up(MODE_PRIM_RX)
        self.clear_irq()

        if self.pipe0_read_addr is not None:
            self.reg_write_bytes(RX_ADDR_P0, self.pipe0_read_addr)

        self.flush_rx()
        self.flush_tx()
        self.ce(1)
        utime.sleep_us(130)

    def stop_listening(self):
        self.ce(0)
        self.flush_tx()
        self.flush_rx()

    # returns True if any data available to recv
    def any(self):
        return not bool(self.reg_read(FIFO_STATUS) & RX_EMPTY)

    def recv(self):
        # get the data
        self.cs(0)
        self.spi.readinto(self.buf, R_RX_PAYLOAD)
        buf = self.spi.read(self.payload_size)
        self.cs(1)
        # clear RX ready flag
        self.reg_write(STATUS, RX_DR)

        return buf

    # blocking wait for tx complete
    def send(self, buf, timeout=500):
        self.send_start(buf)
        start = utime.ticks_ms()
        result = None
        while result is None and utime.ticks_diff(utime.ticks_ms(), start) < timeout:
            result = self.send_done()  # 1 == success, 2 == fail

        self.flush_tx()

        if result is None:
            # timed out, cancel sending and power down the module
            self.shutdown()
            raise OSError("timed out")

        if result == 2:
            raise OSError("send failed")

    # Puts a single message into the fifo
    def put_tx_buf(self, buf: bytes):
        self.cs(0)
        self.spi.readinto(self.buf, W_TX_PAYLOAD)
        self.spi.write(buf)
        if len(buf) < self.payload_size:
            self.spi.write(b"\x00" * (self.payload_size - len(buf)))  # pad out data
        self.cs(1)

    # non-blocking tx
    def send_start(self, buf: bytes):
        # power up
        self.power_up(MODE_PRIM_TX)
        # send the data
        self.put_tx_buf(buf)

        # enable the chip so it can send the data
        self.ce(1)
        utime.sleep_us(15)  # needs to be >10us
        self.ce(0)

    # returns None if send still in progress, 1 for success, 2 for fail
    def send_done(self):
        status = self.read_status()
        if not (status & (TX_DS | MAX_RT)):
            return None  # tx not finished

        # either finished or failed: get and clear status flags, power down
        self.shutdown()
        return 1 if status & TX_DS else 2

    def send_no_ack(self, buf: bytes):
        # power up
        self.reg_write(FEATURE, EN_DYN_ACK)  # enable noack packet
        self.power_up(MODE_PRIM_TX)
        # send the data
        self.cs(0)
        self.spi.readinto(self.buf, W_TX_PAYLOAD_NOACK)
        self.spi.write(buf)
        if len(buf) < self.payload_size:
            self.spi.write(b"\x00" * (self.payload_size - len(buf)))  # pad out data
        self.cs(1)

        # enable the chip so it can send the data
        self.ce(1)
        utime.sleep_us(15)  # needs to be >10us
        self.ce(0)

        # wait for send to complete
        while True:
            utime.sleep_us(500)
            status = self.read_status()
            if status & TX_DS:  # when ACK is disabled, TX_DS is set unconditionally
                break

        self.shutdown()

    # can be used for out-of-order shutdown
    def shutdown(self):
        # assert CE low (or CSN... I'm getting confused... anyway it should stop sending)
        self.ce(0)

        # flush buffers
        self.flush_tx()
        self.flush_rx()

        # disable cont. carrier mode
        self.reg_write(RF_SETUP, self.reg_read(RF_SETUP) & ~(CONT_WAVE | PLL_LOCK))

        # clear flags
        self.clear_irq()

        # power down
        self.reg_write(CONFIG, self.reg_read(CONFIG) & ~PWR_UP)

    def start_constant_carrier_output(self):
        # power up
        self.power_up(MODE_PRIM_TX)

        self.reg_write(RF_SETUP, self.reg_read(RF_SETUP) | CONT_WAVE | PLL_LOCK)  # preserves previous power setup
        # preserves previous channel setup

        self.ce(1)  # BLAST IT!

    # actually it's Received Power Detector... receiving is possible bellow the treshold
    def detect_carrier(self) -> bool:
        return bool(self.reg_read(RPD))
