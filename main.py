import select
import sys
from interactive import menu, default_rx, default_tx
from demo import demo
from const import DEVICE_REVISION


def main():
    print("")
    print(f" == NRF + Pico proto 0{DEVICE_REVISION}; FW ver 0.2 == ")
    print("Press the any key to drop to REPL")
    print("Starting demo mode in 20 seconds")

    p = select.poll()
    p.register(sys.stdin, select.POLLIN)
    events = p.poll(20000)  # 20 sec
    if events:
        # any key pressed, dropping to repl
        sys.stdin.read(1)
        print("The any key was pressed! Dropping to REPL")
        print('type "menu()" for the interactive menu!')
        print('type "demo()" for demo mode!')
        print()
        return

    # the any key was not pressed, starting demo mode
    print("Demo mode!")
    demo()


if __name__ == "__main__":
    main()
