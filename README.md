# fsi6-joystick

Turn a FlySky iBUS or SBUS receiver into a **USB joystick on Linux**, so you can fly
FPV/drone simulators (Liftoff, Velocidrone, DRL, FPV.SkyDive, …) with your FS-i6
(or any transmitter whose receiver speaks iBUS/SBUS).

```
FS-i6 ))) receiver (iBUS/SBUS) --> USB-UART adapter --> fsi6-joystick --> /dev/input/jsX --> simulator
```

The script reads the receiver's serial stream and creates a virtual joystick named
**"FlySky FS-i6 Joystick"** through `uinput`. Sticks and switches all show up as analog
axes, which is what drone sims expect. You then calibrate it in the sim.

> **Linux only.** The virtual joystick uses the Linux `uinput` subsystem. It does not work
> on Windows or macOS.

---

## ⚠️ Read this first if you use SBUS: the signal is INVERTED

**SBUS is an inverted UART signal.** Idle is LOW and the bits are flipped compared to a
normal serial port. A standard USB-UART adapter (CP2102, CH340, FT232, Arduino) expects a
normal, non-inverted signal. **If you connect SBUS straight to the adapter's RX pin, you
won't get any valid frames.** Bytes do arrive, but they're garbage, so the tool reports:

```
data received (... SBUS try: N bytes) but no valid frames:
  - SBUS must be inverted before the adapter RX
```

You must invert the signal **before** it reaches the adapter's RX. Pick one of these:

| Option | How |
|---|---|
| **Use iBUS instead** (easiest) | FlySky receivers (FS-iA6B, FS-iA10B) have an iBUS servo port that is a normal, non-inverted UART. No inverter needed. |
| **FT232R adapter** | Use FTDI's `FT_PROG` tool to set *Invert RXD* in the chip's EEPROM. After that, wire SBUS straight in. |
| **Transistor inverter** | One NPN transistor (2N3904/BC547) + a 10 kΩ base resistor + a 10 kΩ pull-up to 3.3 V/5 V on the collector. SBUS → base, collector → adapter RX. |
| **Logic inverter IC** | One gate of a 74HC04 / 74HC14 (or SN74LVC1G04). SBUS → input, output → adapter RX. |
| **Receiver with an "uninverted SBUS" pad** | Some receivers (often labeled for F4 flight controllers) expose a non-inverted pad. Use that one. |

SBUS also uses an odd serial format: **100000 baud, 8 data bits, even parity, 2 stop bits
(8E2)**. The script sets this up itself. You don't need to change any port settings.

---

## Hardware

- FlySky FS-i6 (or FS-i6X, or any transmitter) bound to a receiver that outputs **iBUS** or **SBUS**
  - FS-iA6B: use the **iBUS servo** port (not the sensor port)
  - Set the transmitter's output to iBUS/SBUS (FS-i6: *System → RX Setup → Output mode*, set the serial output to *i-BUS* or *S.BUS*)
- A USB-UART adapter (CP2102, CH340, FT232…) **or an Arduino Uno used as one**
- Wiring: receiver **signal → adapter RX**, **GND → GND**, and 5 V to power the receiver

### Using an Arduino Uno as the adapter

Connect **RESET to GND**. This holds the ATmega328P in reset, so its USB chip acts as a
plain USB-serial bridge. Then connect the receiver signal to **pin 1 (TX)**, not pin 0.
In this mode the pin labels are from the 328P's point of view. The port is `/dev/ttyACM0`.

---

## Installation

### Option A: prebuilt executable

Download `fsi6-joystick-linux-x86_64.tar.gz` from the
[Releases](../../releases) page:

```bash
tar xzf fsi6-joystick-linux-x86_64.tar.gz
./fsi6-joystick /dev/ttyUSB0
```

### Option B: run from source

```bash
# Arch / CachyOS
sudo pacman -S python-pyserial python-evdev
# Debian / Ubuntu
sudo apt install python3-serial python3-evdev
# or with pip
pip install -r requirements.txt

./fsi6_joystick.py /dev/ttyUSB0
```

### Permissions (one-time)

You need read/write access to the serial port and to `/dev/uinput`:

```bash
sudo modprobe uinput
echo 'KERNEL=="uinput", MODE="0660", GROUP="input", OPTIONS+="static_node=uinput"' \
  | sudo tee /etc/udev/rules.d/99-uinput.rules
sudo usermod -aG input,uucp $USER    # uucp on Arch; dialout on Debian/Ubuntu
# log out and back in
```

To load `uinput` at boot: `echo uinput | sudo tee /etc/modules-load.d/uinput.conf`

---

## Usage

```bash
fsi6-joystick                               # reconnect to the last device, or pick one from a menu
fsi6-joystick --select                      # always show the port picker
fsi6-joystick /dev/ttyUSB0                  # auto-detect iBUS / SBUS
fsi6-joystick /dev/ttyUSB0 -p ibus          # force iBUS
fsi6-joystick /dev/ttyUSB0 -p sbus          # force SBUS (signal must be inverted, see above!)
fsi6-joystick /dev/ttyUSB0 -m 1,2,3,4,5,6   # only 6 axes (stock 6-channel FS-i6)
fsi6-joystick /dev/ttyUSB0 -i 2             # invert axis #2
fsi6-joystick /dev/ttyUSB0 -b 5,6           # also expose CH5, CH6 as buttons (>1500 µs = on)
fsi6-joystick /dev/ttyUSB0 --failsafe-center  # on signal loss: center sticks, throttle low
```

### Choosing the serial port

If you run it without a port, it reconnects to the last device you used, as long as that
device is plugged in. USB adapters are matched by VID/PID and serial number, so it still
works when the adapter comes back as a different `ttyUSBn`/`ttyACMn`. If the device isn't
connected, or you pass `--select`, a small menu opens instead:

```
FS-i6 joystick emulator - select the serial port

 1. /dev/ttyACM0   Arduino (www.arduino.cc)  2341:0043  (last used)
 2. /dev/ttyUSB0   Silicon Labs CP2102 USB to UART Bridge Controller  10c4:ea60

 Up/Down select   Enter connect   1-9 quick pick   a show all ports   q quit
```

The list refreshes every second, so you can plug the adapter in while the menu is open.
`a` also shows the on-board `/dev/ttyS*` ports, which are hidden by default. The last
device is saved in `~/.config/fsi6-joystick/last_device.json` every time a port opens
successfully, including one you name on the command line.

Example of the live display. It shows every channel in µs, a position bar, the axis value, the frame
rate and the link status (`OK` / `FAILSAFE` / `NO SIGNAL`):

```
FlySky FS-i6 joystick emulator  [iBUS @ /dev/ttyACM0]  -> /dev/input/event27
status: OK         rate: 142.9 Hz   frames: 15023    bad: 0

 Roll       -> X         1500us  [--------------#---------------]   1024
 Pitch      -> Y         1500us  [--------------#---------------]   1024
 Throttle   -> Z         1000us  [#-------------|---------------]      0
 Yaw        -> RX        1500us  [--------------#---------------]   1024
 ...
```

### Options

| Option | Default | Description |
|---|---|---|
| `port` | last device / menu | Serial device, e.g. `/dev/ttyUSB0`, `/dev/ttyACM0` |
| `-s, --select` | off | Show the port picker even if the last device is connected |
| `-p, --protocol` | `auto` | `auto`, `ibus` or `sbus` |
| `-m, --map` | `1,…,10` | Channels (1-based) mapped in order to axes X, Y, Z, RX, RY, RZ, THR, RUD, WHL, GAS, BRK, MISC (max 12) |
| `-i, --invert` | (none) | Axis positions (1-based) to invert |
| `-b, --buttons` | (none) | Channels to also expose as buttons |
| `-n, --names` | `Roll,Pitch,Throttle,Yaw` | Display names for CH1, CH2, … (FS-i6 default AETR order) |
| `--failsafe-center` | off | On failsafe or no signal: center sticks, throttle low (default: hold last values) |
| `--timeout` | `0.5` | Seconds without frames before `NO SIGNAL` |
| `--name` | `FlySky FS-i6 Joystick` | Joystick device name |
| `--vid`, `--pid` | `0x1209` / `0x4F54` | USB vendor/product ID reported by the virtual device |
| `-q, --quiet` | off | No live display |

### Check that it works

```bash
jstest /dev/input/js0        # from the joystick / linuxconsoletools package
```

Then calibrate the axes in the simulator's controller settings. Proton/Steam games see
the joystick like a normal device.

---

## Troubleshooting

| Message / symptom | Cause |
|---|---|
| `no data at all on the port` | Wiring: signal must go to the adapter **RX** with a common GND. Is the receiver powered and bound (solid LED)? Arduino: RESET→GND, signal on pin 1. |
| `data received … but no valid frames` with SBUS | **The SBUS signal isn't inverted.** See the [SBUS section](#️-read-this-first-if-you-use-sbus-the-signal-is-inverted). |
| `data received … but no valid frames` with iBUS | The receiver outputs PPM/PWM instead of iBUS, or you're on the iBUS **sensor** port instead of the servo port. |
| `cannot create uinput device` | `sudo modprobe uinput` and set up the udev rule above. |
| `cannot open /dev/ttyUSB0: Permission denied` | Add yourself to `uucp` (Arch) or `dialout` (Debian/Ubuntu). |
| High `bad:` frame counter | Electrical noise or a loose GND. Keep wires short. |

---

## Building the executable yourself

```bash
pip install -r requirements.txt pyinstaller
pyinstaller --onefile --name fsi6-joystick fsi6_joystick.py
# -> dist/fsi6-joystick
```

Pushing a tag `vX.Y.Z` runs the GitHub Action that builds the executable and attaches it
to a GitHub release.

---

## License

[MIT](LICENSE)
