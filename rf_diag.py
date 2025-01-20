import utime
from feedback import FeederBacker
from nrf24l01 import NRF24L01


def channel_scan(
        nrf: NRF24L01,
        feeder_backer: FeederBacker,
        chgen: callable = range(126),
        iterations: int = 500,
        interval_ms: int = 10
) -> dict:
    interval_us = interval_ms * 1000

    results = {}

    for ch in chgen:
        feeder_backer.led2.off()  # red off
        feeder_backer.led3.on()  # green on
        print("Probing channel", ch, "...")
        print(f"\033[K\r0/{iterations}", end="")  # initial
        detections = 0

        nrf.set_channel(ch)
        nrf.start_listening()
        utime.sleep_ms(2)

        last_detection = None
        last_update = utime.ticks_ms()
        for i in range(iterations):
            start = utime.ticks_us()

            detection = nrf.detect_carrier()
            if detection:

                if detections == 0:
                    feeder_backer.led2.on()  # red on
                    feeder_backer.led3.off()  # green off

                detections += 1

                # print only each 100ms to prevent spam
                now = utime.ticks_ms()
                if utime.ticks_diff(now, last_update) > 100:
                    print(f"\033[K\r{detections}/{iterations}", end="")
                    last_update = now

            if last_detection != detection:
                feeder_backer.led_activity(detection)
                last_detection = detection

            utime.sleep_us(interval_us - utime.ticks_diff(utime.ticks_us(), start))

        # end of channel test
        print(f"\033[K\r{detections}/{iterations}")
        print(f"Estimated free air-time: {(1 - (detections / iterations)) * 100}%")

        results[ch] = detections
        feeder_backer.led_activity.off()
        nrf.stop_listening()

        # pause between channels
        feeder_backer.led1.on()
        utime.sleep_ms(25)
        feeder_backer.led1.off()

    return results
