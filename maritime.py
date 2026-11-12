#!/usr/bin/env python3
"""Talks to the maritime board over its binary protocol (details at https://sw.avbotz.com/maritime/protocol).

    ./maritime.py                interactive shell (type help)
    ./maritime.py COMMAND ...    one shell command, e.g. ./maritime.py monitor
"""

import argparse
import cmd
import math
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

# As include/protocol.h
PROTOCOL_VERSION = 3
MSG_THRUSTERS = 0x01
MSG_TORPEDO = 0x02
MSG_DROPPER = 0x03
MSG_GRABBER = 0x04
MSG_SERVO = 0x05
MSG_BOOTLOADER = 0x06
MSG_STATE = 0x81
STATE = struct.Struct("<BBfIII")
STATE_KILLED = 1 << 0
STATE_DEPTH_VALID = 1 << 1
SERVOS = {"dropper": 0, "grabber": 1, "shooter": 2}
BOOTLOADER_MAGIC = 0x544F4F42
NUM_THRUSTERS = 8

# CONFIG_CDC_ACM_SERIAL_PID in prj.conf, Zephyr's VID
USB_ID = (0x2FE3, 0x0004)
# How often a held thruster command is resent: well inside the board's 500 ms [s]
HOLD_PERIOD = 0.1
# How often to look for the board while it is gone [s]
RECONNECT_PERIOD = 0.5
# No frame is near this long: more without a delimiter is noise
MAX_RECORD = 256


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_at = 0

    def finish():
        nonlocal code_at
        out[code_at] = len(out) - code_at
        code_at = len(out)
        out.append(0)

    for byte in data:
        if len(out) - code_at == 0xFF:
            finish()
        if byte == 0:
            finish()
        else:
            out.append(byte)
    out[code_at] = len(out) - code_at
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        block = data[i + 1 : i + code]
        if code == 0 or i + code > len(data) or 0 in block:
            return None
        out += block
        i += code
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(msg_type: int, payload: bytes) -> bytes:
    return b"\0" + cobs_encode(bytes([msg_type]) + payload) + b"\0"


def decode_frame(record: bytes) -> tuple[int, bytes] | None:
    """(type, payload) of what came between two delimiters, or None if garbled."""
    body = cobs_decode(record)
    if not body:
        return None
    return body[0], body[1:]


@dataclass
class State:
    killed: bool
    depth: float | None
    uptime_ms: int
    command_timeouts: int
    rx_errors: int

    def __str__(self):
        depth = f"{self.depth:.3f} m" if self.depth is not None else "unknown"
        return (
            f"{'killed' if self.killed else 'released'}, depth {depth}, "
            f"uptime {self.uptime_ms / 1000:.1f} s, command timeouts {self.command_timeouts}, "
            f"board rx errors {self.rx_errors}"
        )


def find_port() -> str | None:
    # deploy/udev/99-maritime.rules in thalassic makes /dev/maritime
    if os.path.exists("/dev/maritime"):
        return "/dev/maritime"
    for port in list_ports.comports():
        if (port.vid, port.pid) == USB_ID:
            return port.device
    return None


class Board:
    """The link to the board, kept up by a thread of its own: it reconnects when the
    board comes back, and resends the held thruster command."""

    def __init__(self, port: str | None, report):
        self._port = port
        self._report = report
        self._serial = None
        # Guards _serial against the shell and the thread both sending
        self._lock = threading.Lock()
        self._received = b""
        self._skip_record = True
        self._last_reason = None
        self._closing = threading.Event()
        self._new_state = threading.Condition()
        self.state: State | None = None
        # The thruster command being resent, if any
        self.hold: tuple[float, ...] | None = None
        self.watching = False
        self.bad_frames = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def connected(self) -> bool:
        return self._serial is not None

    def wait_for_state(self, timeout: float) -> State | None:
        with self._new_state:
            self._new_state.wait_for(lambda: self.state is not None, timeout)
            return self.state

    def send(self, msg_type: int, payload: bytes = b""):
        with self._lock:
            if self._serial is None:
                raise ConnectionError("the board is not connected")
            try:
                self._serial.write(encode_frame(msg_type, payload))
            except (serial.SerialException, OSError) as e:
                raise ConnectionError(f"write failed: {e}") from e

    def thrust(self, thrusts):
        """Sends `thrusts`, then keeps resending them until changed."""
        self.send(MSG_THRUSTERS, struct.pack(f"<{NUM_THRUSTERS}f", *thrusts))
        self.hold = tuple(thrusts)

    def close(self):
        if self.hold is not None and self.connected:
            try:
                self.thrust([0.0] * NUM_THRUSTERS)
            except ConnectionError:
                pass
        self._closing.set()
        self._thread.join()
        with self._lock:
            if self._serial is not None:
                self._serial.close()

    def _run(self):
        next_hold = 0.0
        while not self._closing.is_set():
            if self._serial is None:
                self._connect()
                if self._serial is None:
                    self._closing.wait(RECONNECT_PERIOD)
                continue
            try:
                data = self._serial.read(max(1, self._serial.in_waiting))
            except (serial.SerialException, OSError):
                self._disconnect("the board is gone")
                continue
            self._receive(data)
            hold = self.hold
            if hold is not None and time.monotonic() >= next_hold:
                next_hold = time.monotonic() + HOLD_PERIOD
                try:
                    self.send(MSG_THRUSTERS, struct.pack(f"<{NUM_THRUSTERS}f", *hold))
                except ConnectionError:
                    self._disconnect("the board is not taking writes")

    def _connect(self):
        port = self._port or find_port()
        if port is None:
            reason = "no maritime board found: waiting for one (or name its port with --port)"
        else:
            try:
                link = serial.Serial(port, timeout=0.02, exclusive=True)
                link.reset_input_buffer()
            except (serial.SerialException, OSError) as e:
                hint = " (is sub_low running?)" if "lock" in str(e) else ""
                reason = f"cannot open {port}{hint}: {e}; retrying"
            else:
                with self._lock:
                    self._serial = link
                self._received = b""
                self._skip_record = True
                self._last_reason = None
                self._report(f"connected to {port}")
                return
        self._report_once(reason)

    def _report_once(self, reason: str):
        """Reports `reason` unless it was the last thing reported this way."""
        if reason != self._last_reason:
            self._report(reason)
            self._last_reason = reason

    def _disconnect(self, reason: str):
        with self._lock:
            if self._serial is not None:
                self._serial.close()
            self._serial = None
        self.state = None
        if self.hold is not None:
            reason += "; thruster command dropped"
        self.hold = None
        self._report(reason)

    def _receive(self, data: bytes):
        *records, self._received = (self._received + data).split(b"\0")
        for record in records:
            # The first is most likely the end of a frame cut off when the port opened
            if self._skip_record:
                self._skip_record = False
            elif record:
                self._handle(record)
        if len(self._received) > MAX_RECORD:
            if is_text(self._received):
                self._report_once(
                    "the board sends text, not frames: is its firmware older than protocol 3?"
                )
            self._received = b""
            self._skip_record = True

    def _handle(self, record: bytes):
        # A debug build's logs share the link: show them rather than count them. No
        # frame is all text, since the board's types (0x81 on) are not.
        if is_text(record):
            self._report("board: " + record.decode("ascii").strip())
            return
        frame = decode_frame(record)
        if frame is None:
            self.bad_frames += 1
            return
        msg_type, payload = frame
        if msg_type != MSG_STATE or not payload:
            self.bad_frames += 1
            return
        if payload[0] != PROTOCOL_VERSION:
            self._report_once(
                f"the board speaks protocol {payload[0]}, this script {PROTOCOL_VERSION}"
            )
            return
        if len(payload) != STATE.size:
            self.bad_frames += 1
            return

        _, flags, depth, uptime_ms, timeouts, rx_errors = STATE.unpack(payload)
        state = State(
            killed=bool(flags & STATE_KILLED),
            depth=depth if flags & STATE_DEPTH_VALID else None,
            uptime_ms=uptime_ms,
            command_timeouts=timeouts,
            rx_errors=rx_errors,
        )
        last = self.state
        if last is not None and state.uptime_ms < last.uptime_ms:
            self._report("the board restarted")
        if last is None or state.killed != last.killed:
            self._report(
                "kill switch: " + ("killed" if state.killed else "released, thrusters obeyed")
            )
        with self._new_state:
            self.state = state
            self._new_state.notify_all()
        if self.watching:
            self._report(str(state))


def is_text(data: bytes) -> bool:
    return all(32 <= b < 127 or b in b"\r\n\t\x1b" for b in data)


def parse_thrust(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or abs(value) > 1:
        raise ValueError(f"thrust {text} is not between -1 and 1")
    return value


def parse_actuator(args: list[str]) -> bytes:
    if len(args) != 2 or args[0] not in ("0", "1") or args[1] not in ("open", "close", "1", "0"):
        raise ValueError("usage: <0|1> <open|close>")
    return bytes([int(args[0]), args[1] in ("open", "1")])


class Shell(cmd.Cmd):
    intro = "maritime: type help or ? for commands, Ctrl+D to quit"
    prompt = "maritime> "

    def __init__(self, board: Board):
        super().__init__()
        self.board = board
        # The last command failed
        self.failed = False

    def onecmd(self, line):
        self.failed = False
        try:
            return super().onecmd(line)
        except (ValueError, ConnectionError) as e:
            print(e)
            self.failed = True
            return False

    def default(self, line):
        raise ValueError(f"unknown command: {line} (type help)")

    def emptyline(self):
        # Rather than repeat the last command, which may be bootloader
        return False

    def postcmd(self, stop, line):
        self.prompt = "maritime (thrust held)> " if self.holding() else "maritime> "
        return stop

    def holding(self) -> bool:
        return self.board.hold is not None and any(self.board.hold)

    def do_status(self, arg):
        """status: the board's last state, and the link's counts"""
        state = self.board.wait_for_state(1.0)
        if state is None:
            raise ConnectionError("no state from the board")
        print(state)
        print(f"bad frames received {self.board.bad_frames}")
        if self.board.hold is not None:
            print("holding thrust " + " ".join(f"{t:g}" for t in self.board.hold))

    def do_monitor(self, arg):
        """monitor: print every state the board sends (5 Hz) until Ctrl+C"""
        self.board.watching = True
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print()
        finally:
            self.board.watching = False

    def do_thrust(self, arg):
        """thrust <t0> ... <t7>: hold all eight thrusters (each -1 to 1; the board caps at 0.6)"""
        args = arg.split()
        if len(args) != NUM_THRUSTERS:
            raise ValueError(f"usage: thrust <t0> ... <t{NUM_THRUSTERS - 1}>")
        self.board.thrust([parse_thrust(a) for a in args])

    def do_all(self, arg):
        """all <t>: hold every thruster at t"""
        self.board.thrust([parse_thrust(arg)] * NUM_THRUSTERS)

    def do_one(self, arg):
        """one <id> <t>: hold thruster id (0-7) at t and the others at 0"""
        args = arg.split()
        if len(args) != 2 or not args[0].isdigit() or int(args[0]) >= NUM_THRUSTERS:
            raise ValueError(f"usage: one <0-{NUM_THRUSTERS - 1}> <t>")
        thrusts = [0.0] * NUM_THRUSTERS
        thrusts[int(args[0])] = parse_thrust(args[1])
        self.board.thrust(thrusts)

    def do_stop(self, arg):
        """stop: hold every thruster at 0"""
        self.board.thrust([0.0] * NUM_THRUSTERS)

    def do_torpedo(self, arg):
        """torpedo <0|1> <open|close>"""
        self.board.send(MSG_TORPEDO, parse_actuator(arg.split()))

    def do_dropper(self, arg):
        """dropper <0|1> <open|close>"""
        self.board.send(MSG_DROPPER, parse_actuator(arg.split()))

    def do_servo(self, arg):
        """servo <dropper|grabber|shooter> <pulse_us>: raw pulse, clamped to the servo's range"""
        args = arg.split()
        if (
            len(args) != 2
            or args[0] not in SERVOS
            or not args[1].isdigit()
            or int(args[1]) > 0xFFFF
        ):
            raise ValueError("usage: servo <dropper|grabber|shooter> <pulse_us>")
        self.board.send(MSG_SERVO, struct.pack("<BH", SERVOS[args[0]], int(args[1])))

    def do_grabber(self, arg):
        """grabber <open|close|0-1>: open, close, or how closed (0 open, 1 closed)"""
        positions = {"open": 0.0, "close": 1.0}
        try:
            closed = positions[arg] if arg in positions else float(arg)
        except ValueError:
            closed = math.nan
        if not 0 <= closed <= 1:
            raise ValueError("usage: grabber <open|close|0-1>")
        self.board.send(MSG_GRABBER, struct.pack("<f", closed))

    def do_bootloader(self, arg):
        """bootloader: reboot into the UF2 bootloader (stops the thrusters first)"""
        self.board.send(MSG_BOOTLOADER, struct.pack("<I", BOOTLOADER_MAGIC))
        self.board.hold = None

    def do_quit(self, arg):
        """quit: stop the thrusters and exit"""
        return True

    def do_EOF(self, arg):
        print()
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Talk to the maritime board.",
        epilog="Commands: status, monitor, thrust, all, one, stop, torpedo, dropper, grabber, servo, "
        "bootloader. With none, an interactive shell.",
    )
    parser.add_argument(
        "--port", "-p", help="serial port (default: /dev/maritime, else found by USB ID)"
    )
    parser.add_argument("command", nargs=argparse.REMAINDER, help="one shell command to run")
    args = parser.parse_args()

    board = Board(args.port, lambda message: print(message, flush=True))
    shell = Shell(board)
    try:
        if not args.command:
            shell.cmdloop()
            return
        # A command needs the link up: wait for it, as long as it takes to enumerate
        deadline = time.monotonic() + 3
        while not board.connected and time.monotonic() < deadline:
            time.sleep(0.05)
        shell.onecmd(" ".join(args.command))
        if shell.failed:
            sys.exit(1)
        if shell.holding():
            # The thrusters run only while this script resends the command
            print("holding thrust until Ctrl+C")
            shell.do_monitor("")
    except KeyboardInterrupt:
        print()
    finally:
        board.close()


if __name__ == "__main__":
    main()
