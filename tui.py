import sys

from micropython import const
import select


class Dialog:
    KEY_TYPE = const(1)
    KEY_NAME = const(2)
    KEY_KEY = const(3)
    KEY_CHOICE_OPTS = const(4)
    KEY_INPUT_PARSER = const(5)

    TYPE_ACTION = const(1)
    TYPE_CHOICE = const(2)
    TYPE_CHECKBOX = const(3)
    TYPE_INPUT = const(4)

    KEY_UNDEF = const(0)
    KEY_UP = const(1)
    KEY_DOWN = const(2)
    KEY_LEFT = const(3)
    KEY_RIGHT = const(4)
    KEY_SELECT = const(5)
    KEY_EXIT = const(6)
    KEY_REDRAW = const(7)

    def __init__(self, title: str):
        self._title = title
        self._items = []

        self._selected = 0
        self._line_length = 0
        self._result = None
        self._active = False

    def add_action(self, name: str, key: any):
        self._items.append({
            self.KEY_TYPE: self.TYPE_ACTION,
            self.KEY_NAME: name,
            self.KEY_KEY: key,
        })

    def add_choice(self, name: str, key: any, options: list[tuple[any, str]]):
        self._items.append({
            self.KEY_TYPE: self.TYPE_CHOICE,
            self.KEY_NAME: name,
            self.KEY_KEY: key,
            self.KEY_CHOICE_OPTS: options,
        })

    def add_checkbox(self, name: str, key: any):
        self._items.append({
            self.KEY_TYPE: self.TYPE_CHECKBOX,
            self.KEY_NAME: name,
            self.KEY_KEY: key,
        })

    def add_input(self, name: str, key: any, parser: callable = None):
        self._items.append({
            self.KEY_TYPE: self.TYPE_INPUT,
            self.KEY_NAME: name,
            self.KEY_KEY: key,
            self.KEY_INPUT_PARSER: parser
        })

    def _header(self):
        print("\033[2J\033[0;0H", end="")  # clear screen # goto 0,0
        print("  == NRF + Pico proto 01; FW ver 0.1 == ")
        print("  == Interactive mode == ")
        print()

    def present(self, options: dict = None) -> dict:
        # validate
        if len(self._items) == 0:
            raise Exception("items undefined")

        # calculate some stuff
        longest_name = 0
        for item in self._items:
            longest_name = max(len(item[self.KEY_NAME]), longest_name)

        self._line_length = longest_name + 6
        if self._line_length < 35:
            self._line_length = 35

        # reset stuff
        if options:
            self._result = options.copy()
        else:
            self._result = {}

        self._active = True  # will be set false, when an action is executed
        self._selected = 0

        # workaround to empty key buffer:
        self._readkey(False)  # micropython stdin does not implement flush()

        # print permanent stuff
        print("\x1b[?25l", end="")  # turn off cursor
        self._header()
        print(self._title)

        for _ in self._items:
            # make space for items
            print()

        # print dynamic stuff

        while self._active:
            self._redraw()
            while True:
                try:
                    key = self._parsekey(self._readkey())
                except KeyboardInterrupt:  # same as exit
                    self._active = False
                    break

                if key == self.KEY_REDRAW:
                    self._header()
                    print(self._title)
                    break

                if key != self.KEY_UNDEF:
                    self._exec_key(key)
                    break

        print("\x1b[?25h", end="")  # turn on cursor
        return self._result

    def _get_opt_index(self, item_idx: int, val: any):
        item = self._items[item_idx]
        assert item[self.KEY_TYPE] == self.TYPE_CHOICE
        idx = -1
        if val is not None:
            for i, opt in enumerate(item[self.KEY_CHOICE_OPTS]):
                if opt[0] == val:
                    idx = i
                    break
        return idx

    def _redraw(self):
        print("\033[5;0H", end="")  # move bellow the title
        for idx, item in enumerate(self._items):
            print("\033[K\r", end="")  # clear line
            if self._selected == idx:
                print("> ", end="")
            else:
                print("  ", end="")

            name = item[self.KEY_NAME]
            print(name, end="")

            if item[self.KEY_TYPE] == self.TYPE_ACTION:
                print()
                continue

            val_s = ""
            if item[self.KEY_TYPE] == self.TYPE_CHOICE:
                val = self._result.get(item[self.KEY_KEY], None)
                opt_idx = self._get_opt_index(idx, val)

                if opt_idx != -1:
                    val_s = f"<{item[self.KEY_CHOICE_OPTS][opt_idx][1]}>"
                else:
                    val_s = "<  >"

            if item[self.KEY_TYPE] == self.TYPE_CHECKBOX:
                if self._result.get(item[self.KEY_KEY], False):
                    val_s = "[X]"
                else:
                    val_s = "[ ]"

            if item[self.KEY_TYPE] == self.TYPE_INPUT:
                val = self._result.get(item[self.KEY_KEY], "")
                val_s = f"({val})"

            spacing = self._line_length - len(name) - len(val_s)
            if spacing > 0:
                print("." * spacing, end="")

            print(val_s)

    def _readkey(self, block=True) -> bytes:
        p = select.poll()
        p.register(sys.stdin, select.POLLIN)
        keybuf = b""
        while True:
            events = p.poll(25)  # 25ms
            if not events:
                if len(keybuf) > 0 or (not block):
                    return keybuf
                continue

            keybuf += sys.stdin.read(1)

    def _parsekey(self, keybuf: bytes) -> int:
        if keybuf[0] == 0x0a or keybuf[0] == 0x0d:  # return
            return self.KEY_SELECT

        if keybuf[0] == 0x20:  # space
            return self.KEY_SELECT

        if keybuf[0] == 0x1b:  # maybe start of an escape sequence

            # esc
            if len(keybuf) == 1:
                return self.KEY_EXIT

            # arrow keys
            if len(keybuf) == 3 and keybuf[1] == 0x5b:
                if keybuf[2] == 0x41:
                    return self.KEY_UP
                if keybuf[2] == 0x42:
                    return self.KEY_DOWN
                if keybuf[2] == 0x43:
                    return self.KEY_RIGHT
                if keybuf[2] == 0x44:
                    return self.KEY_LEFT
                if keybuf[2] == 0x5A:  # shift-tab
                    return self.KEY_UP

            # page up / page down
            if len(keybuf) == 4 and keybuf[1] == 0x5b and keybuf[3] == 0x7e:
                if keybuf[2] == 0x35:
                    return self.KEY_UP
                if keybuf[2] == 0x36:
                    return self.KEY_DOWN

        if keybuf[0] == 0x04:  # ctrl-d
            return self.KEY_EXIT

        if keybuf[0] == 0x09:  # tab
            return self.KEY_DOWN

        if keybuf[0] == 0x7f:  # backspace
            return self.KEY_EXIT

        if keybuf[0] == 0x12:  # ctrl-r
            return self.KEY_REDRAW

        return self.KEY_UNDEF

    def _exec_key(self, key: int):
        if key == self.KEY_UP:
            if self._selected > 0:
                self._selected -= 1
            else:
                self._selected = len(self._items) - 1
            return

        if key == self.KEY_DOWN:
            if self._selected < len(self._items) - 1:
                self._selected += 1
            else:
                self._selected = 0
            return

        if key == self.KEY_EXIT:
            self._active = False
            return

        item = self._items[self._selected]
        result_key = item[self.KEY_KEY]

        if item[self.KEY_TYPE] == self.TYPE_ACTION:
            if key == self.KEY_SELECT:
                self._result[result_key] = True
                self._active = False
                return

        if item[self.KEY_TYPE] == self.TYPE_CHOICE:
            val = self._result.get(item[self.KEY_KEY], None)
            opt_idx = self._get_opt_index(self._selected, val)

            if opt_idx == -1:
                self._result[item[self.KEY_KEY]] = item[self.KEY_CHOICE_OPTS][0][0]
                return

            max_opt = len(item[self.KEY_CHOICE_OPTS]) - 1
            if key == self.KEY_LEFT:
                opt_idx -= 1
                if opt_idx < 0:
                    opt_idx = max_opt

            if key == self.KEY_RIGHT or key == self.KEY_SELECT:
                opt_idx += 1
                if opt_idx > max_opt:
                    opt_idx = 0

            self._result[item[self.KEY_KEY]] = item[self.KEY_CHOICE_OPTS][opt_idx][0]

        if item[self.KEY_TYPE] == self.TYPE_CHECKBOX:
            if key in (self.KEY_SELECT, self.KEY_LEFT, self.KEY_RIGHT):
                self._result[result_key] = not self._result.get(result_key, False)
                return

        if item[self.KEY_TYPE] == self.TYPE_INPUT:
            user_input = self._read_freetext(item[self.KEY_INPUT_PARSER])
            if user_input is not None:
                self._result[result_key] = user_input

            # TODO: this is a hack, should handle screen lifecycle better
            self._header()
            print(self._title)

    def _read_freetext(self, parser: callable = None) -> any:
        print("\x1b[?25h", end="")  # turn on cursor
        while True:
            print()
            print("Enter new value: (Ctrl-C to cancel)")
            try:
                user_input = input("? ")
            except KeyboardInterrupt:
                # cancel
                print("\x1b[?25l", end="")  # turn off cursor
                return None

            # workaround to empty key buffer:
            self._readkey(False)  # micropython stdin does not implement flush()

            if parser:  # parser should raise error on validation errors, or it can convert the value... or both
                try:
                    user_input = parser(user_input, self._result)
                except Exception as e:
                    print(f"INVALID! ({str(e)})")
                    continue

            print("\x1b[?25l", end="")  # turn off cursor
            return user_input
