#!/usr/bin/env python3
"""
fsi6_joystick.py - turn an iBUS / SBUS receiver on a serial port into a
virtual FlySky FS-i6 style USB joystick (Linux, uinput).

    receiver (iBUS/SBUS) --> USB-UART adapter --> this script --> /dev/input/jsX
                                                                  (Liftoff, Velocidrone,
                                                                   DRL, FPV.SkyDive, ...)

Requirements:  python-evdev, pyserial, write access to /dev/uinput
"""

import argparse
import json
import os
import select
import signal
import sys
import termios
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  sudo pacman -S python-pyserial   (or: pip install pyserial)")
from serial.tools import list_ports
try:
    from evdev import UInput, AbsInfo, ecodes as e
except ImportError:
    sys.exit("python-evdev missing:  sudo pacman -S python-evdev   (or: pip install evdev)")


# --------------------------------------------------------------------------
# Protocol parsers
# --------------------------------------------------------------------------

class IBusParser:
    """FlySky iBUS: 115200 8N1, 32-byte frames every ~7 ms.

    0x20 0x40 | 14 x uint16 LE channels | uint16 LE checksum (0xFFFF - sum of bytes 0..29)
    """
    NAME = "iBUS"
    BAUD = 115200
    FRAME_LEN = 32
    NUM_CH = 14

    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        """Feed raw bytes, yield (channels[list], failsafe[bool]) per good frame."""
        self.buf += data
        while len(self.buf) >= self.FRAME_LEN:
            if self.buf[0] != 0x20 or self.buf[1] != 0x40:
                del self.buf[0]
                continue
            frame = self.buf[:self.FRAME_LEN]
            chk = 0xFFFF - (sum(frame[:30]) & 0xFFFF)
            if chk != (frame[30] | frame[31] << 8):
                self.bad_frames += 1
                del self.buf[0]
                continue
            del self.buf[:self.FRAME_LEN]
            ch = [frame[2 + 2 * i] | (frame[3 + 2 * i] << 8) for i in range(self.NUM_CH)]
            # FS-iA6B on newer firmware packs extra bits in the high nibble; mask it off
            ch = [c & 0x0FFF for c in ch]
            yield ch, False


class SBusParser:
    """Futaba SBUS: 100000 baud 8E2, *inverted*, 25-byte frames.

    0x0F | 22 bytes = 16 x 11-bit channels (LSB first) | flags | 0x00 (footer)
    flags: bit0 ch17, bit1 ch18, bit2 frame lost, bit3 failsafe
    Raw 172..1811 is mapped to 988..2012 us (same as Betaflight).
    """
    NAME = "SBUS"
    BAUD = 100000
    FRAME_LEN = 25
    NUM_CH = 16

    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buf += data
        while len(self.buf) >= self.FRAME_LEN:
            if self.buf[0] != 0x0F:
                del self.buf[0]
                continue
            frame = self.buf[:self.FRAME_LEN]
            # footer is 0x00 for plain SBUS; some receivers send 0x04/0x14/0x24/0x34 (SBUS2)
            if frame[24] not in (0x00, 0x04, 0x14, 0x24, 0x34):
                self.bad_frames += 1
                del self.buf[0]
                continue
            del self.buf[:self.FRAME_LEN]
            bits = int.from_bytes(frame[1:23], "little")
            ch = []
            for i in range(self.NUM_CH):
                raw = (bits >> (11 * i)) & 0x7FF
                ch.append(int(round((raw - 992) * 5 / 8 + 1500)))
            flags = frame[23]
            failsafe = bool(flags & 0x08)
            yield ch, failsafe


# --------------------------------------------------------------------------
# Virtual FS-i6 joystick
# --------------------------------------------------------------------------

# FS-i6 / FS-SM100 style: sticks + switches all show up as analog axes,
# which is what every drone sim expects (you map/calibrate them in the sim).
AXIS_CODES = [
    e.ABS_X, e.ABS_Y, e.ABS_Z, e.ABS_RX, e.ABS_RY, e.ABS_RZ,
    e.ABS_THROTTLE, e.ABS_RUDDER, e.ABS_WHEEL, e.ABS_GAS, e.ABS_BRAKE, e.ABS_MISC,
]   # (HAT axes deliberately skipped - SDL turns those into d-pads, not axes)
AXIS_NAMES = ["X", "Y", "Z", "RX", "RY", "RZ", "THR", "RUD", "WHL", "GAS", "BRK", "MISC"]

# FS-i6 default channel order is AETR (Mode 2): CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw
DEFAULT_CH_NAMES = "Roll,Pitch,Throttle,Yaw"
AXIS_MIN, AXIS_MAX = 0, 2047          # 11-bit, like a real RC joystick dongle
US_MIN, US_MAX = 1000, 2000           # RC pulse range in microseconds

# Generic pid.codes VID/PID. Sims don't need specific IDs (you calibrate in-game);
# override with --vid/--pid if you want to mimic a specific dongle.
DEFAULT_VID, DEFAULT_PID = 0x1209, 0x4F54


def us_to_axis(us, invert=False):
    v = (us - US_MIN) * (AXIS_MAX - AXIS_MIN) / (US_MAX - US_MIN) + AXIS_MIN
    v = max(AXIS_MIN, min(AXIS_MAX, int(round(v))))
    return AXIS_MAX - v + AXIS_MIN if invert else v


class VirtualFSi6:
    def __init__(self, n_axes, name, vid, pid, buttons):
        self.n_axes = n_axes
        absinfo = AbsInfo(value=(AXIS_MIN + AXIS_MAX) // 2, min=AXIS_MIN, max=AXIS_MAX,
                          fuzz=0, flat=0, resolution=0)
        caps = {e.EV_ABS: [(AXIS_CODES[i], absinfo) for i in range(n_axes)]}
        # SDL/most sims only treat a device as a joystick if it has at least one button
        self.n_buttons = max(1, buttons)
        caps[e.EV_KEY] = [e.BTN_TRIGGER + i for i in range(self.n_buttons)]
        self.ui = UInput(caps, name=name, vendor=vid, product=pid, version=0x0111,
                         bustype=e.BUS_USB)
        self.last = [None] * n_axes
        self.last_btn = [None] * self.n_buttons

    def update(self, axis_values, button_values):
        changed = False
        for i, v in enumerate(axis_values):
            if v != self.last[i]:
                self.ui.write(e.EV_ABS, AXIS_CODES[i], v)
                self.last[i] = v
                changed = True
        for i, b in enumerate(button_values):
            if b != self.last_btn[i]:
                self.ui.write(e.EV_KEY, e.BTN_TRIGGER + i, int(b))
                self.last_btn[i] = b
                changed = True
        if changed:
            self.ui.syn()

    @property
    def device(self):
        return self.ui.device.path if self.ui.device else "?"

    def close(self):
        self.ui.close()


# --------------------------------------------------------------------------
# Console display
# --------------------------------------------------------------------------

CSI = "\x1b["


def bar(us, width=30):
    frac = (max(US_MIN, min(US_MAX, us)) - US_MIN) / (US_MAX - US_MIN)
    pos = int(round(frac * (width - 1)))
    mid = (width - 1) // 2
    cells = ["-"] * width
    cells[mid] = "|"
    cells[pos] = "#"
    return "".join(cells)


def draw(state, args, vjoy):
    lines = []
    status = state["status"]
    colour = {"OK": "32", "FAILSAFE": "31", "NO SIGNAL": "33", "WAITING": "33"}[status]
    lines.append(f"{CSI}1mFS-i6 joystick emulator{CSI}0m  "
                 f"[{state['proto']} @ {args.port}]  -> {vjoy.device}")
    lines.append(f"status: {CSI}{colour};1m{status:<9}{CSI}0m  "
                 f"rate: {state['rate']:5.1f} Hz   frames: {state['frames']:<8} "
                 f"bad: {state['bad']:<6}")
    lines.append("")
    ch = state["channels"]
    for out_i, src in enumerate(args.map):
        us = ch[src] if src < len(ch) else 1500
        inv = "inv" if out_i in args.invert else "   "
        lines.append(f" {args.names[src]:<10} -> {AXIS_NAMES[out_i]:<4} {inv} "
                     f"{us:5d}us  [{bar(us)}]  {state['axes'][out_i]:5d}")
    if args.buttons:
        lines.append("")
        btn = "  ".join(f"{args.names[src]}:{'ON ' if b else 'off'}"
                        for src, b in zip(args.buttons, state["buttons"]))
        lines.append(" " + btn)
    lines.append("")
    lines.append(" Ctrl+C to quit")
    sys.stdout.write(f"{CSI}H" + "\n".join(l + f"{CSI}K" for l in lines) + f"{CSI}J")
    sys.stdout.flush()


# --------------------------------------------------------------------------
# Serial port picker + remembered last device
# --------------------------------------------------------------------------

CONFIG_FILE = os.path.join(os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config"),
                           "fsi6-joystick", "last_device.json")


def serial_ports(show_all=False):
    """Connected serial ports; legacy on-board /dev/ttyS* without hardware are hidden."""
    ports = [p for p in list_ports.comports() if show_all or p.hwid != "n/a"]
    return sorted(ports, key=lambda p: p.device)


def port_label(p):
    desc = " ".join(x for x in (p.manufacturer, p.product) if x) or p.description
    if desc == p.name or desc == "n/a":
        desc = ""
    ids = f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None else ""
    return f"{p.device:<14} {desc}  {ids}".rstrip()


def same_device(p, last):
    """USB adapters can come back as a different ttyUSBn/ttyACMn; match them by
    VID/PID/serial number when available, by device path otherwise."""
    if last.get("serial_number") and p.serial_number:
        return (p.vid, p.pid, p.serial_number) == \
            (last.get("vid"), last.get("pid"), last["serial_number"])
    return p.device == last.get("device")


def load_last_device():
    try:
        with open(CONFIG_FILE) as f:
            last = json.load(f)
        return last if isinstance(last, dict) and last.get("device") else None
    except (OSError, ValueError):
        return None


def save_last_device(port):
    real = os.path.realpath(port)
    info = {"device": port, "vid": None, "pid": None, "serial_number": None}
    for p in list_ports.comports():
        if p.device == real:
            info.update(vid=p.vid, pid=p.pid, serial_number=p.serial_number)
            break
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, "w") as f:
            json.dump(info, f, indent=2)
    except OSError:
        pass                        # remembering the port is a convenience only


def can_open(port):
    try:
        serial.Serial(port).close()
        return None
    except serial.SerialException as ex:
        return str(ex)


def pick_port(last=None, note=""):
    """Arrow-key menu on the terminal; refreshes every second so a freshly plugged-in
    adapter shows up. Returns the device path, or exits on q / Esc / Ctrl+C."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    # no line buffering/echo; Ctrl+C arrives as a byte instead of a signal
    new[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
    show_all, sel_dev, chosen = False, None, None
    sys.stdout.write(f"{CSI}?1049h{CSI}?25l")     # alternate screen, hide cursor
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, new)
        while chosen is None:
            ports = serial_ports(show_all)
            devs = [p.device for p in ports]
            if sel_dev not in devs:
                sel_dev = next((p.device for p in ports if last and same_device(p, last)),
                               devs[0] if devs else None)
            sel = devs.index(sel_dev) if sel_dev else 0

            lines = [f"{CSI}1mFS-i6 joystick emulator - select the serial port{CSI}0m", ""]
            if note:
                lines += [f" {CSI}33m{note}{CSI}0m", ""]
            if not ports:
                lines.append(" (no serial ports found - plug in the USB-UART adapter)")
            for i, p in enumerate(ports):
                mark = "  (last used)" if last and same_device(p, last) else ""
                text = f"{i + 1:>2}. {port_label(p)}{mark}"
                lines.append(f" {CSI}7m{text}{CSI}0m" if i == sel else f" {text}")
            lines += ["", f" {CSI}2mUp/Down select   Enter connect   1-9 quick pick   "
                          f"a {'hide' if show_all else 'show'} all ports   q quit{CSI}0m"]
            sys.stdout.write(f"{CSI}H" + "\n".join(l + f"{CSI}K" for l in lines) + f"{CSI}J")
            sys.stdout.flush()

            if not select.select([fd], [], [], 1.0)[0]:
                continue                                  # timeout -> rescan ports
            key = os.read(fd, 16).decode(errors="ignore")
            if key in ("q", "Q", "\x1b", "\x03", "\x04"):
                break
            if key in ("\x1b[A", "\x1bOA", "k") and ports:
                sel_dev = devs[(sel - 1) % len(devs)]
            elif key in ("\x1b[B", "\x1bOB", "j") and ports:
                sel_dev = devs[(sel + 1) % len(devs)]
            elif key in ("\r", "\n") and ports:
                chosen = sel_dev
            elif key.isdigit() and 1 <= int(key) <= len(devs):
                chosen = devs[int(key) - 1]
            elif key in ("a", "A"):
                show_all = not show_all
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write(f"{CSI}?25h{CSI}?1049l")
        sys.stdout.flush()
    if chosen is None:
        sys.exit(130)
    return chosen


def choose_port(force_menu):
    """No port on the command line: reconnect to the last used device if it's
    plugged in, otherwise (or with --select) show the picker."""
    last = load_last_device()
    note = ""
    if last and not force_menu:
        match = next((p for p in serial_ports(True) if same_device(p, last)), None)
        if match:
            err = can_open(match.device)
            if err is None:
                print(f"using last device {match.device}   (--select to choose another)")
                return match.device
            note = f"last device {match.device} cannot be opened: {err}"
        else:
            note = f"last device {last['device']} is not connected"
    if not sys.stdin.isatty():
        ports = "\n".join("  " + port_label(p) for p in serial_ports()) or "  (none)"
        sys.exit(f"no serial port given and no terminal for the picker. Ports:\n{ports}")
    return pick_port(last, note)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def parse_list(s):
    return [int(x) for x in s.split(",") if x.strip()] if s else []


def open_serial(port, proto):
    if proto == "sbus":
        return serial.Serial(port, 100000, bytesize=serial.EIGHTBITS,
                             parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO,
                             timeout=0.02)
    return serial.Serial(port, 115200, bytesize=serial.EIGHTBITS,
                         parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                         timeout=0.02)


def autodetect(port, seconds=1.5):
    """Try iBUS then SBUS; return (protocol or None, {proto: bytes_received})."""
    seen = {}
    for proto, cls in (("ibus", IBusParser), ("sbus", SBusParser)):
        try:
            ser = open_serial(port, proto)
        except serial.SerialException as ex:
            sys.exit(f"cannot open {port}: {ex}")
        p = cls()
        good, nbytes = 0, 0
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            data = ser.read(256)
            nbytes += len(data)
            for _ in p.feed(data):
                good += 1
            if good >= 5:
                ser.close()
                return proto, seen
        seen[proto] = nbytes
        ser.close()
    return None, seen


def main():
    ap = argparse.ArgumentParser(
        description="Read iBUS/SBUS from a serial port and expose it as a FlySky FS-i6 "
                    "style joystick for drone simulators.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  %(prog)s                                     # reconnect to the last device, or pick one
  %(prog)s --select                            # always show the port picker
  %(prog)s /dev/ttyUSB0                        # auto-detect protocol
  %(prog)s /dev/ttyUSB0 -p ibus                # FS-iA6B / FS-iA10B iBUS servo port
  %(prog)s /dev/ttyUSB0 -p sbus                # SBUS (needs an inverted signal!)
  %(prog)s /dev/ttyUSB0 -m 1,2,3,4,5,6         # only 6 axes (stock FS-i6)
  %(prog)s /dev/ttyUSB0 -i 2                   # invert axis #2 (Y)
  %(prog)s /dev/ttyUSB0 -b 5,6                 # also expose CH5,CH6 as buttons (>1500us = on)
""")
    ap.add_argument("port", nargs="?",
                    help="serial device, e.g. /dev/ttyUSB0 or /dev/ttyACM0 (default: the "
                         "last used device if connected, otherwise a picker menu)")
    ap.add_argument("-s", "--select", action="store_true",
                    help="show the serial port picker even if the last device is connected")
    ap.add_argument("-p", "--protocol", choices=["auto", "ibus", "sbus"], default="auto")
    ap.add_argument("-m", "--map", default="1,2,3,4,5,6,7,8,9,10",
                    help="comma-separated 1-based channels -> axes X,Y,Z,RX,RY,RZ,THR,RUD,... "
                         "(default: 1..10 = FS-i6 10ch firmware; use 1,2,3,4,5,6 for a "
                         "stock 6ch FS-i6)")
    ap.add_argument("-i", "--invert", default="",
                    help="comma-separated 1-based axis positions to invert")
    ap.add_argument("-b", "--buttons", default="",
                    help="comma-separated 1-based channels to also expose as buttons")
    ap.add_argument("--name", default="FlySky FS-i6 Joystick", help="device name")
    ap.add_argument("--vid", type=lambda x: int(x, 0), default=DEFAULT_VID)
    ap.add_argument("--pid", type=lambda x: int(x, 0), default=DEFAULT_PID)
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="seconds without frames before 'NO SIGNAL' (default 0.5)")
    ap.add_argument("-n", "--names", default=DEFAULT_CH_NAMES,
                    help="comma-separated display names for CH1, CH2, ...; unnamed channels "
                         f"show as 'Channel N' (default: {DEFAULT_CH_NAMES} = FS-i6 AETR)")
    ap.add_argument("--failsafe-center", action="store_true",
                    help="on failsafe/no signal: center sticks and put throttle (CH3) low "
                         "instead of holding last values")
    ap.add_argument("-q", "--quiet", action="store_true", help="no live display")
    args = ap.parse_args()

    args.map = [c - 1 for c in parse_list(args.map)]
    args.invert = {c - 1 for c in parse_list(args.invert)}
    args.buttons = [c - 1 for c in parse_list(args.buttons)]
    names = [n.strip() for n in args.names.split(",") if n.strip()]
    args.names = names[:16] + [f"Channel {i + 1}" for i in range(len(names), 16)]
    if not 1 <= len(args.map) <= len(AXIS_CODES):
        sys.exit(f"--map needs 1..{len(AXIS_CODES)} channels")
    if any(c < 0 or c >= 16 for c in args.map + args.buttons):
        sys.exit("channel numbers must be 1..16")
    if len(args.buttons) > 16:
        sys.exit("at most 16 buttons")

    if args.port is None:
        args.port = choose_port(args.select)

    proto = args.protocol
    if proto == "auto":
        print(f"auto-detecting protocol on {args.port} ...")
        proto, seen = autodetect(args.port)
        if not proto:
            if not any(seen.values()):
                sys.exit("no data at all on the port -> wiring problem, not protocol:\n"
                         "  - receiver signal must go to the adapter's RX, plus common GND\n"
                         "  - receiver powered and bound (LED solid)?\n"
                         "  - Arduino Uno as adapter: tie RESET to GND and connect the\n"
                         "    signal to pin 1 (TX), not pin 0")
            sys.exit(f"data received (iBUS try: {seen.get('ibus', 0)} bytes, SBUS try: "
                     f"{seen.get('sbus', 0)} bytes) but no valid frames:\n"
                     "  - SBUS must be inverted before the adapter RX\n"
                     "  - receiver must be set to output iBUS/SBUS (not PPM/PWM)")
        print(f"detected {proto.upper()}")
    parser = IBusParser() if proto == "ibus" else SBusParser()

    try:
        ser = open_serial(args.port, proto)
    except serial.SerialException as ex:
        sys.exit(f"cannot open {args.port}: {ex}")
    save_last_device(args.port)

    try:
        vjoy = VirtualFSi6(len(args.map), args.name, args.vid, args.pid, len(args.buttons))
    except (PermissionError, OSError) as ex:
        sys.exit(f"cannot create uinput device: {ex}\n"
                 "fix:  sudo modprobe uinput  and give yourself access to /dev/uinput, e.g.\n"
                 '  echo \'KERNEL=="uinput", MODE="0660", GROUP="input", OPTIONS+="static_node=uinput"\' '
                 "| sudo tee /etc/udev/rules.d/99-uinput.rules\n"
                 "  sudo usermod -aG input $USER   (then log out/in)")

    center = 1500
    state = {
        "proto": parser.NAME, "status": "WAITING", "rate": 0.0, "frames": 0, "bad": 0,
        "channels": [center] * parser.NUM_CH,
        "axes": [us_to_axis(center)] * len(args.map),
        "buttons": [False] * len(args.buttons),
    }

    last_frame_t = 0.0
    rate_t0, rate_n = time.monotonic(), 0
    next_draw = 0.0
    if not args.quiet:
        sys.stdout.write(f"{CSI}?25l{CSI}2J")   # hide cursor, clear screen

    def apply(ch, safe_mode):
        if safe_mode and args.failsafe_center:
            ch = [center] * len(ch)
            if len(ch) > 2:
                ch[2] = US_MIN          # throttle low (AETR)
        axes = [us_to_axis(ch[src], i in args.invert) for i, src in enumerate(args.map)]
        btns = [ch[src] > 1500 for src in args.buttons]
        state["axes"], state["buttons"] = axes, btns
        vjoy.update(axes, btns)

    # Ctrl+C / SIGTERM just set a flag so the loop exits and cleanup always runs.
    # (A PyInstaller one-file build gets SIGINT twice: from the terminal and
    # forwarded by its bootloader - a KeyboardInterrupt would hit the cleanup.)
    stop = False

    def request_stop(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while not stop:
            data = ser.read(max(1, ser.in_waiting))
            now = time.monotonic()
            for ch, failsafe in parser.feed(data):
                state["frames"] += 1
                rate_n += 1
                last_frame_t = now
                state["channels"] = ch
                state["status"] = "FAILSAFE" if failsafe else "OK"
                apply(ch, failsafe)

            if state["frames"] and now - last_frame_t > args.timeout \
                    and state["status"] != "NO SIGNAL":
                state["status"] = "NO SIGNAL"
                apply(state["channels"], True)

            if now - rate_t0 >= 1.0:
                state["rate"] = rate_n / (now - rate_t0)
                rate_t0, rate_n = now, 0
            state["bad"] = parser.bad_frames

            if not args.quiet and now >= next_draw:
                draw(state, args, vjoy)
                next_draw = now + 0.05
    except serial.SerialException as ex:
        print(f"\nserial error: {ex}", file=sys.stderr)
    finally:
        # late/duplicate signals must not kill us during cleanup or interpreter
        # shutdown (Python resets Python-level handlers to default at exit)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if not args.quiet:
            sys.stdout.write(f"{CSI}?25h\n")
        vjoy.close()
        ser.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:      # Ctrl+C during startup / protocol auto-detect
        sys.exit(130)
