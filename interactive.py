import sys
import ustruct as struct
import utime

from feedback import FeederBacker
import nrf24l01
from micropython import const

from machine import Pin, SPI

from const import IRQ_PIN
from nperf import nperf, DIRECTION_RX, DIRECTION_TX

from transfer import tx_irq, tx_poll, rx_irq, rx_poll
from rf_diag import channel_scan, echo_test
from pingpong import pingpong
from hw_diag import led_test, interrupt_test

CFG_RADIO_CHANNEL = "c"
CFG_RADIO_SPEED = "s"
CFG_RADIO_CRC = "r"
CFG_RADIO_POWER = "p"
CFG_RADIO_RETR_COUNT = "t"
CFG_RADIO_RETR_DELAY = "d"

DEFAULT_RADIO_CONFIG = {
    CFG_RADIO_CHANNEL: 8,
    CFG_RADIO_SPEED: nrf24l01.SPEED_250K,
    CFG_RADIO_CRC: 1,
    CFG_RADIO_POWER: nrf24l01.POWER_3,
    CFG_RADIO_RETR_DELAY: nrf24l01.ARD_1750US,
    CFG_RADIO_RETR_COUNT: 8,
}

CFG_RX_BELL = const(1)
CFG_RX_IRQ = const(2)

DEFAULT_RX_CONFIG = {
    CFG_RX_BELL: False,
    CFG_RX_IRQ: True
}

CFG_TX_IRQ = const(1)
CFG_TX_INTERVAL = const(2)

DEFAULT_TX_CONFIG = {
    CFG_TX_IRQ: True,
    CFG_TX_INTERVAL: 2000,
}

CFG_SCAN_ITERATIONS = const(1)
CFG_SCAN_INTERVAL = const(2)
CFG_SCAN_START = const(3)
CFG_SCAN_END = const(4)
CFG_SCAN_REPORT_STYLE = const(5)

SCAN_REPORT_SIMPLE = const(1)
SCAN_REPORT_FULL = const(2)
SCAN_REPORT_FANCY = const(3)
SCAN_REPORT_RAW = const(4)

DEFAULT_SCAN_CONFIG = {
    CFG_SCAN_ITERATIONS: 500,  # 5s
    CFG_SCAN_INTERVAL: 10,
    CFG_SCAN_START: 0,
    CFG_SCAN_END: 84,
    CFG_SCAN_REPORT_STYLE: SCAN_REPORT_SIMPLE,
}

CFG_PINGPONG_HAS_SERVE = const(1)

DEFAULT_PINGPONG_CONFIG = {
    CFG_PINGPONG_HAS_SERVE: False
}

CFG_NPERF_DIRECTION = const(1)

DEFAULT_NPERF_CONFIG = {
    CFG_NPERF_DIRECTION: DIRECTION_RX,
}

last_cnt = 0  # used by parse_and_print

# Addresses are in little-endian format. They correspond to big-endian
PIPE_ADDRESSES = (b'\xe1\xf0\xf0\xf0\xf0', b'\xd2\xf0\xf0\xf0\xf0')

PAYLOAD_SIZE = const(16)


def print_radio_config(config: dict):
    print("  Channel:", config[CFG_RADIO_CHANNEL])
    print("  CRC:", {0: "off", 1: "8bit", 2: "16bit"}[config[CFG_RADIO_CRC]])
    print("  Speed:",
          {nrf24l01.SPEED_250K: "250K", nrf24l01.SPEED_1M: "1M", nrf24l01.SPEED_2M: "2M"}[config[CFG_RADIO_SPEED]])
    print("  Power:",
          {nrf24l01.POWER_0: "MIN", nrf24l01.POWER_1: "LOW", nrf24l01.POWER_2: "HIGH", nrf24l01.POWER_3: "MAX"}[
              config[CFG_RADIO_POWER]])
    if config[CFG_RADIO_RETR_COUNT] == 0:
        print("  Retransmit: off")
    else:
        print("  Retransmit: ", config[CFG_RADIO_RETR_COUNT])
        delay_strings = {
            nrf24l01.ARD_250US: "250us",
            nrf24l01.ARD_500US: "500us",
            nrf24l01.ARD_750US: "750us",
            nrf24l01.ARD_1000US: "1000us",
            nrf24l01.ARD_1250US: "1250us",
            nrf24l01.ARD_1500US: "1500us",
            nrf24l01.ARD_1750US: "1750us",
            nrf24l01.ARD_2000US: "2000us",
            nrf24l01.ARD_2250US: "2250us",
            nrf24l01.ARD_2500US: "2500us",
            nrf24l01.ARD_2750US: "2750us",
            nrf24l01.ARD_3000US: "3000us",
            nrf24l01.ARD_3250US: "3250us",
            nrf24l01.ARD_3500US: "3500us",
            nrf24l01.ARD_3750US: "3750us",
            nrf24l01.ARD_4000US: "4000us",
        }
        print("  Retransmit delay:", delay_strings[config[CFG_RADIO_RETR_DELAY]])


def init_nrf(config: dict) -> nrf24l01.NRF24L01:
    print(" == nrf init ==")
    csn = Pin(14, mode=Pin.OUT, value=1)
    ce = Pin(13, mode=Pin.OUT, value=0)
    spidev = SPI(1, sck=Pin(10), mosi=Pin(15), miso=Pin(8))
    print("SPI config:", spidev)
    print("rx addr:", PIPE_ADDRESSES[0])
    print("tx addr:", PIPE_ADDRESSES[1])
    print("radio config:")
    print_radio_config(config)

    nrf = nrf24l01.NRF24L01(spidev, csn, ce,
                            payload_size=PAYLOAD_SIZE,
                            channel=config[CFG_RADIO_CHANNEL]
                            )

    nrf.set_power_speed(config[CFG_RADIO_POWER], config[CFG_RADIO_SPEED])
    nrf.set_crc(config[CFG_RADIO_CRC])
    nrf.setup_retr(config[CFG_RADIO_RETR_DELAY], config[CFG_RADIO_RETR_COUNT])
    nrf.open_tx_pipe(PIPE_ADDRESSES[1])
    nrf.open_rx_pipe(1, PIPE_ADDRESSES[0])

    print(" == nrf init completed! ==")
    return nrf


def parse_and_print_msg(msg: bytes) -> tuple[bool, bool, int]:  # preamble, last_fail, cnt
    global last_cnt
    if msg[:3] == b'abc':
        print("  PREAMBLE: OK")
    else:
        print("  PREAMBLE: FAIL")
        return False, False, 0

    cnt = struct.unpack("<H", msg[3:5])[0]
    miss = cnt - last_cnt != 1
    last_cnt = cnt
    last_fail = bool(msg[5])  # Note: True means failure!

    if miss:
        print("  CNT:", cnt, "!!!")
    else:
        print("  CNT:", cnt)

    if last_fail:
        print("  LAST TX: FAIL")
    else:
        print("  LAST TX: OK")

    return True, last_fail, cnt


def message(counter: int, last_fail: bool) -> bytes:
    return b'abc' + struct.pack("<H", counter) + (b'\x01' if last_fail else b'\x00')


def run_simple_tx(radio_conf: dict, tx_conf: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)
    interval = tx_conf.get(CFG_TX_INTERVAL, 2000)
    print("interval:", interval)

    if tx_conf[CFG_TX_IRQ]:
        print("Sending messages (IRQ mode)")
        fun = tx_irq
    else:
        print("Sending messages (POLL mode)")
        fun = tx_poll

    counter = 0
    last_fail = False

    def generator():
        nonlocal counter
        nonlocal last_fail
        while True:
            msg = message(counter, last_fail)
            counter += 1
            print("sending message:")
            parse_and_print_msg(msg)
            yield msg
            utime.sleep_ms(interval)

    def callback(success: bool, err_msg: str):
        nonlocal last_fail
        nonlocal feeder_backer
        last_fail = not success
        if success:
            feeder_backer.led2.off()
            feeder_backer.led3.on()
            print(" ACK: OK")
        else:
            feeder_backer.led2.on()
            feeder_backer.led3.off()
            print(f" ACK: FAIL ({err_msg})")

        lost_packets, retr_count = nrf.observe_tx()
        print(f" Retransmissions: {retr_count}")

    try:
        fun(nrf, feeder_backer, generator, callback)
    finally:
        nrf.shutdown()


def run_simple_rx(radio_conf: dict, rx_conf: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)
    bell = rx_conf.get(CFG_RX_BELL, False)
    print("bell:", bell)

    if rx_conf[CFG_RX_IRQ]:
        print("Waiting for messages (IRQ mode)")
        fun = rx_irq
    else:
        print("Waiting for messages (POLL mode)")
        fun = rx_poll

    _last_cnt = 0
    try:
        for msg in fun(nrf, feeder_backer):
            print("received message:")
            if bell:
                print("\a", end="")
            preamble_ok, last_fail, cnt = parse_and_print_msg(msg)

            if last_fail or (not preamble_ok):
                # indicate some failure
                feeder_backer.led2.on()
                feeder_backer.led3.off()
            else:
                # indicate success
                feeder_backer.led2.off()
                feeder_backer.led3.on()

            if cnt != (_last_cnt + 1):
                feeder_backer.led1.on()  # indicate some missing packets
            else:
                feeder_backer.led1.off()
            _last_cnt = cnt

    finally:
        nrf.shutdown()


def unmodulated_carrier_tx(radio_conf: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)
    nrf.stop_listening()
    nrf.start_constant_carrier_output()
    feeder_backer.led_activity.on()
    print("now transmitting un-modulated carrier wave...")
    try:
        while True:
            pass
    finally:
        feeder_backer.led_activity.off()
        nrf.shutdown()
        print("Transmission stopped.")


def simple_carrier_rx(radio_conf: dict, feeder_backer: FeederBacker):
    # TODO: for some reason, receiving a valid packed locks this up in high state
    # Maybe data needs to be read?

    nrf = init_nrf(radio_conf)
    nrf.set_auto_ack(False)  # don't want to send ack
    nrf.flush_rx()
    nrf.start_listening()
    last_state = False
    first = True
    irq_pin = Pin(IRQ_PIN, pull=Pin.PULL_UP, mode=Pin.IN)
    print("Detecting carrier wave...")
    try:
        while True:
            utime.sleep_us(500)  # 0.5ms

            # since we are in listening mode, we may receive messages as well, but we don't care now
            if not irq_pin.value():  # irq would be on falling edge
                # clear flags
                nrf.flush_rx()
                nrf.clear_irq()
                feeder_backer.led1.on()

            state = nrf.detect_carrier()
            if last_state != state:
                if state and first:
                    feeder_backer.led2.on()
                    first = False

                feeder_backer.led_activity(state)

                print(f"\033[K\rCR: {state} ", end="")
                last_state = state

    finally:
        nrf.shutdown()


def run_channel_scan(radio_conf: dict, scan_config: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)

    iterations = scan_config[CFG_SCAN_ITERATIONS]
    channel_time = scan_config[CFG_SCAN_INTERVAL] * iterations
    total_time = (channel_time * (scan_config[CFG_SCAN_END] - scan_config[CFG_SCAN_START])) / 1000  # seconds

    print("iterations:", iterations)

    print(f"Probing each channel for {channel_time}ms for carrier wave...")
    print(f"This will take {total_time:.2f} seconds!")

    try:
        results = channel_scan(
            nrf,
            feeder_backer,
            range(scan_config[CFG_SCAN_START], scan_config[CFG_SCAN_END] + 1),
            iterations,
            scan_config[CFG_SCAN_INTERVAL]
        )
    finally:
        nrf.shutdown()

    feeder_backer.reset()

    print("Scan completed!")

    if scan_config[CFG_SCAN_REPORT_STYLE] == SCAN_REPORT_SIMPLE:
        # just display channels with minimal noise
        min_val = min(results.values())
        free_airtime = (1 - (min_val / iterations)) * 100
        print('Channel(s) with most available air-time:')
        print(f'(that is {free_airtime:3.2f}%)')
        for ch, val in results.items():
            if val == min_val:
                print(f"- Channel {ch:3} ({free_airtime:.2f}%)")

    if scan_config[CFG_SCAN_REPORT_STYLE] == SCAN_REPORT_FULL:
        ordered_vals = list(set(results.values()))
        ordered_vals.sort()  # asc by default

        print('All channels ordered by free airtime:')
        for rank_val in ordered_vals:
            for ch, val in results.items():
                if val == rank_val:
                    free_airtime = (1 - (val / iterations)) * 100
                    print(f"- Channel {ch:3} ({free_airtime:.2f}%)")

    if scan_config[CFG_SCAN_REPORT_STYLE] == SCAN_REPORT_FANCY:
        min_val = min(results.values())
        print(" ch | air t   | rating       | b | val")
        for ch in range(scan_config[CFG_SCAN_START], scan_config[CFG_SCAN_END] + 1):
            val = results[ch]
            free_airtime = 1 - val / iterations
            free_airtime_percent = free_airtime * 100
            stars = int(free_airtime ** 10 * 12)

            # this only works because we are scanning continuous channels
            ch_before = ch - 1
            ch_after = ch + 1

            best = " "
            if ch_before in results and ch_after in results:
                if (results[ch_before], results[ch], results[ch_after]) == (min_val,) * 3:
                    best = "X"

            print(f"{ch:3} | {free_airtime_percent:6.2f}% |", "*" * stars + " " * (12 - stars), f"| {best} | {val}")

    if scan_config[CFG_SCAN_REPORT_STYLE] == SCAN_REPORT_RAW:
        # lol
        print(results)


def run_pingpong(radio_conf: dict, pingpong_config: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)
    try:
        pingpong(nrf, feeder_backer, pingpong_config[CFG_PINGPONG_HAS_SERVE])
    finally:
        nrf.shutdown()


def run_nperf(radio_conf: dict, nperf_config: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)
    try:
        nperf(nrf, feeder_backer, nperf_config[CFG_NPERF_DIRECTION])
    finally:
        nrf.shutdown()


def run_echo_test(radio_conf: dict, feeder_backer: FeederBacker):
    nrf = init_nrf(radio_conf)
    try:
        echo_test(nrf, feeder_backer)
    finally:
        nrf.shutdown()


def probe_nrf(radio_conf: dict, feeder_backer: FeederBacker):
    feeder_backer.led_builtin.on()
    nrf = None
    try:
        # init
        print("Initializing device...")
        nrf = init_nrf(radio_conf)

        # read status
        print("Reading status...")  # by sending noop
        status = nrf.read_status()
        print(f"  STATUS: {hex(status)}")

        # read a register
        print("Reading some registers...")  # to test communication
        conf = nrf.reg_read(nrf24l01.CONFIG)
        print(f"  CONFIG: {hex(conf)}")
        aw = nrf.reg_read(nrf24l01.SETUP_AW)
        print(f"  SETUP_AW: {hex(aw)}")
        if aw == 0x00:
            raise Exception("0x00 is illegal for SETUP_AW")
        elif aw == 0xff:
            raise Exception("0xff is illegal for SETUP_AW")

        # send with irq
        print("Testing IRQ...")
        nrf.clear_irq()
        p = Pin(IRQ_PIN, pull=Pin.PULL_UP, mode=Pin.IN)

        if not p.value():
            raise Exception("IRQ pin stuck!")

        nrf.send_start(b"\x00\x01")

        # poll the irq pin for irq
        start = utime.ticks_ms()
        pin_val = True
        while pin_val and utime.ticks_diff(utime.ticks_ms(), start) < 50:
            pin_val = p.value()

        if not pin_val:
            print("  Got IRQ!")
            nrf.send_done()
        else:
            raise Exception("Got no IRQ!")

        # shutting down
        print("Shutting down...")
        nrf.shutdown()
        nrf = None

    except Exception as e:
        print(f"FAILURE: {e}")
        feeder_backer.led2.on()
    else:
        print("Success. The nrf device seems to be working!")
        feeder_backer.led3.on()
    finally:
        feeder_backer.led_builtin.off()
        if nrf:
            nrf.shutdown()


def default_rx():
    feeder_backer = FeederBacker()
    run_simple_rx(DEFAULT_RADIO_CONFIG, DEFAULT_RX_CONFIG, feeder_backer)


def default_tx():
    feeder_backer = FeederBacker()
    run_simple_tx(DEFAULT_RADIO_CONFIG, DEFAULT_TX_CONFIG, feeder_backer)


def menu():
    from tui import Dialog

    feeder_backer = FeederBacker()

    def protected_run(fn: callable, *args, **kwargs):
        try:
            fn(*args, **kwargs)
        except KeyboardInterrupt as e:
            sys.print_exception(e)
            print("\nInterrupted. Hit return.")
        except Exception as e:
            sys.print_exception(e)
            print(f"\nCrashed: {str(e)}. Hit return.")
        else:
            print("\nExited. Hit return.")
        input()

    def parse_channel(t: str, _: dict) -> int:
        chint = int(t)
        if 0 <= chint < 126:
            return chint
        else:
            raise Exception("invalid channel")

    def parse_ard_count(t: str, _: dict) -> int:
        rtint = int(t)
        if 0 <= rtint <= 15:
            return rtint
        else:
            raise Exception("invalid count")

    def parse_positive_number(t: str, _: dict) -> int:
        i = int(t)
        if i > 0:
            return i
        else:
            raise Exception("<0")

    ACTION_OK = const(-1)
    ACTION_CANCEL = const(-2)

    # radio config dialog
    config_d = Dialog("Radio config")
    config_d.add_input("Channel", CFG_RADIO_CHANNEL, parse_channel)
    config_d.add_choice("Speed", CFG_RADIO_SPEED,
                        [(nrf24l01.SPEED_250K, "256K"), (nrf24l01.SPEED_1M, "1M"), (nrf24l01.SPEED_2M, "2M")])
    config_d.add_choice("CRC", CFG_RADIO_CRC, [(0, "off"), (1, "8bit"), (2, "16bit")])
    config_d.add_choice("Power", CFG_RADIO_POWER, [
        (nrf24l01.POWER_0, "MIN (-18 dBm)"),
        (nrf24l01.POWER_1, "LOW (-12 dBm)"),
        (nrf24l01.POWER_2, "HIGH (-6 dBm)"),
        (nrf24l01.POWER_3, "MAX (0 dBm)")
    ])
    config_d.add_choice("Auto Retransmit Delay", CFG_RADIO_RETR_DELAY, [
        (nrf24l01.ARD_250US, "250us"),
        (nrf24l01.ARD_500US, "500us"),
        (nrf24l01.ARD_750US, "750us"),
        (nrf24l01.ARD_1000US, "1000us"),
        (nrf24l01.ARD_1250US, "1250us"),
        (nrf24l01.ARD_1500US, "1500us"),
        (nrf24l01.ARD_1750US, "1750us"),
        (nrf24l01.ARD_2000US, "2000us"),
        (nrf24l01.ARD_2250US, "2250us"),
        (nrf24l01.ARD_2500US, "2500us"),
        (nrf24l01.ARD_2750US, "2750us"),
        (nrf24l01.ARD_3000US, "3000us"),
        (nrf24l01.ARD_3250US, "3250us"),
        (nrf24l01.ARD_3500US, "3500us"),
        (nrf24l01.ARD_3750US, "3750us"),
        (nrf24l01.ARD_4000US, "4000us"),
    ])
    config_d.add_input("Auto Retransmit Count", CFG_RADIO_RETR_COUNT, parse_ard_count)
    config_d.add_action("Save", ACTION_OK)
    config_d.add_action("Discard", ACTION_CANCEL)

    # receiver dialog
    rx_d = Dialog("Simple receiver")
    rx_d.add_checkbox("Bell on RX", CFG_RX_BELL)
    rx_d.add_choice("Dev events", CFG_RX_IRQ, [(True, "IRQ"), (False, "POLL")])
    rx_d.add_action("Run", ACTION_OK)
    rx_d.add_action("Cancel", ACTION_CANCEL)

    # transmitter dialog
    tx_d = Dialog("Simple transmitter")
    tx_d.add_choice("Dev events", CFG_TX_IRQ, [(True, "IRQ"), (False, "POLL")])
    tx_d.add_input("Send interval", CFG_TX_INTERVAL, parse_positive_number)
    tx_d.add_action("Run", ACTION_OK)
    tx_d.add_action("Cancel", ACTION_CANCEL)

    # scan dialog
    scan_d = Dialog("Scan channels and measure available air-time")
    scan_d.add_input("Iterations", CFG_SCAN_ITERATIONS, parse_positive_number)
    scan_d.add_input("Interval", CFG_SCAN_INTERVAL, parse_positive_number)
    scan_d.add_input("Start channel", CFG_SCAN_START, parse_positive_number)
    scan_d.add_input("End channel", CFG_SCAN_END, parse_positive_number)
    scan_d.add_choice("Report style", CFG_SCAN_REPORT_STYLE, [
        (SCAN_REPORT_SIMPLE, "simple"),
        (SCAN_REPORT_FANCY, "fancy"),
        (SCAN_REPORT_FULL, "full"),
        (SCAN_REPORT_RAW, "raw")
    ])
    scan_d.add_action("Run", ACTION_OK)
    scan_d.add_action("Cancel", ACTION_CANCEL)

    # pingpong dialog
    pingpong_d = Dialog("Ping-pong")
    pingpong_d.add_checkbox("Has serve", CFG_PINGPONG_HAS_SERVE)
    pingpong_d.add_action("Run", ACTION_OK)
    pingpong_d.add_action("Cancel", ACTION_CANCEL)

    # nperf dialog
    nperf_d = Dialog("nPerf")
    nperf_d.add_choice("Direction", CFG_NPERF_DIRECTION, [
        (DIRECTION_TX, "send"),
        (DIRECTION_RX, "recv")
    ])
    nperf_d.add_action("Run", ACTION_OK)
    nperf_d.add_action("Cancel", ACTION_CANCEL)

    radio_config = DEFAULT_RADIO_CONFIG.copy()
    rx_config = DEFAULT_RX_CONFIG.copy()
    tx_config = DEFAULT_TX_CONFIG.copy()
    scan_config = DEFAULT_SCAN_CONFIG.copy()
    pingpong_config = DEFAULT_PINGPONG_CONFIG.copy()
    nperf_config = DEFAULT_NPERF_CONFIG.copy()

    def action_config():
        nonlocal radio_config
        new_radio_config = config_d.present(radio_config)
        if ACTION_OK in new_radio_config:
            del new_radio_config[ACTION_OK]
            radio_config = new_radio_config

    def action_rx():
        nonlocal rx_config
        new_rx_config = rx_d.present(rx_config)
        if ACTION_OK in new_rx_config:
            del new_rx_config[ACTION_OK]
            rx_config = new_rx_config
            protected_run(run_simple_rx, radio_config, rx_config, feeder_backer)

    def action_tx():
        nonlocal tx_config
        new_tx_config = tx_d.present(tx_config)
        if ACTION_OK in new_tx_config:
            del new_tx_config[ACTION_OK]
            tx_config = new_tx_config
            protected_run(run_simple_tx, radio_config, tx_config, feeder_backer)

    def action_scan():
        nonlocal scan_config
        new_scan_config = scan_d.present(scan_config)
        if ACTION_OK in new_scan_config:
            del new_scan_config[ACTION_OK]
            scan_config = new_scan_config
            protected_run(run_channel_scan, radio_config, scan_config, feeder_backer)

    def action_pingpong():
        nonlocal pingpong_config
        new_pingpong_config = pingpong_d.present(pingpong_config)
        if ACTION_OK in new_pingpong_config:
            del new_pingpong_config[ACTION_OK]
            pingpong_config = new_pingpong_config
            protected_run(run_pingpong, radio_config, pingpong_config, feeder_backer)

    def action_nperf():
        nonlocal nperf_config
        new_nperf_config = nperf_d.present(nperf_config)
        if ACTION_OK in new_nperf_config:
            del new_nperf_config[ACTION_OK]
            nperf_config = new_nperf_config
            protected_run(run_nperf, radio_config, nperf_config, feeder_backer)

    applets = [
        # name, function, protected
        (
            "Configure radio", action_config, False
        ),
        (
            "Probe nRF device", lambda: probe_nrf(radio_config, feeder_backer), True
        ),
        (
            "Run simple receiver", action_rx, False
        ),
        (
            "Run simple transmitter", action_tx, False
        ),
        (
            "Emit un-modulated carrier wave", lambda: unmodulated_carrier_tx(radio_config, feeder_backer), True
        ),
        (
            "Detect carrier wave", lambda: simple_carrier_rx(radio_config, feeder_backer), True
        ),
        (
            "Scan channels for free airtime", action_scan, False
        ),
        (
            "Run ping-pong", action_pingpong, False
        ),
        (
            "Run nPerf", action_nperf, False
        ),
        (
            "Run echo test", lambda: run_echo_test(radio_config, feeder_backer), True
        ),
        (
            "Run IRQ test", lambda: interrupt_test(IRQ_PIN, feeder_backer), True
        ),
        (
            "Run LED test", lambda: led_test(feeder_backer), True
        )
    ]

    # main dialog
    main_d = Dialog("Where do you want to go today?")

    for i, elm in enumerate(applets):
        main_d.add_action(elm[0], i)

    main_d.add_action("Leave menu", ACTION_CANCEL)

    while True:
        feeder_backer.reset()
        feeder_backer.led3.on()  # indicate menu
        result = main_d.present()

        if len(result) == 0:
            # nothing selected
            break

        choice = list(result.keys())[0]

        if choice == ACTION_CANCEL:
            # user choose to exit
            break

        _, func, protect = applets[choice]
        feeder_backer.reset()
        if protect:
            protected_run(func)
        else:
            func()

    feeder_backer.reset()
