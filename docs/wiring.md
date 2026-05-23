# Wiring

This page covers the two physical links the system relies on:

1. **Jetson Orin Nano <-> LD19 LiDAR** (via the supplied CP2102 USB adapter)
2. **Jetson Orin Nano <-> Pixhawk** (via USB, with an optional TELEM2 UART alternative)

Recommended setup is **USB for both** — it is the path that the LDRobot
quickstart and ArduPilot dev docs treat as the primary supported route.

---

## 1. LD19 LiDAR (LDRobot D500 kit)

The development kit ships with a CP2102 USB-to-UART adapter board that
also powers the LiDAR motor from USB VBUS. There is no need to touch the
Jetson GPIO header.

| LD19 pin         | CP2102 adapter | Function                  |
|------------------|----------------|---------------------------|
| `VCC` (5 V)      | `VCC`          | Motor + sensor power      |
| `GND`            | `GND`          | Ground                    |
| `TX` (3.3 V)     | `RX`           | LiDAR -> host serial      |
| `PWM` (optional) | not connected  | Internal speed control    |

**Host side** (Jetson Orin Nano):

1. Plug the CP2102 adapter into any USB port.
2. Verify enumeration: `dmesg | tail`
3. The kernel exposes the LiDAR as `/dev/ttyUSB0` (or `/dev/ttyUSB1` if
   another CP2102 device is already present).
4. Grant your user access without `sudo`:

   ```bash
   sudo usermod -aG dialout $USER
   # log out + back in
   ```

Test the link with:

```bash
python scripts/ld19_dump.py --port /dev/ttyUSB0
```

You should see ~360 points per scan at ~10 Hz.

### Advanced: direct UART to the Jetson 40-pin header

Only do this if you need the USB port for something else.

| Jetson pin | Signal | LD19 pin |
|------------|--------|----------|
| `2`        | `5V`   | `VCC`    |
| `6`        | `GND`  | `GND`    |
| `10`       | `RXD`  | `TX`     |

The LD19 outputs 3.3 V which matches the Jetson, so no level shifter is
needed. Then disable the serial console so Linux doesn't fight you for
the UART:

```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
```

The device path becomes `/dev/ttyTHS1`.

---

## 2. Pixhawk autopilot

### Primary: USB

```
Jetson USB <-> USB-C/Micro-USB cable <-> Pixhawk USB port
```

ArduPilot exposes a MAVLink endpoint at `/dev/ttyACM0` @ `115200` baud.
Verify with:

```bash
python scripts/mavlink_ping.py --connection /dev/ttyACM0
```

You should see a heartbeat within a couple of seconds. The `AUTOPILOT_VERSION`
message confirms ArduPilot is responding.

### Alternative: TELEM2 UART

Use when the USB port is taken (e.g. by the LiDAR adapter).

| Pixhawk TELEM2 | Jetson UART1 (40-pin header) |
|----------------|------------------------------|
| `TX`           | pin `10` (`RXD`)             |
| `RX`           | pin `8`  (`TXD`)             |
| `GND`          | pin `6`  (`GND`)             |
| `5V` (do not)  | (do not back-power the Jetson) |

Set on the autopilot (load via MAVProxy):

```
SERIAL2_PROTOCOL 2
SERIAL2_BAUD 921
```

Disable the Jetson serial console (as above) and connect at:

```bash
python scripts/mavlink_ping.py --connection /dev/ttyTHS1 --baud 921600
```

---

## 3. Power budget

| Item                     | Typical current @ 5 V |
|--------------------------|------------------------|
| Jetson Orin Nano (idle)  | 1.5 A                  |
| Jetson Orin Nano (load)  | 3-4 A                  |
| LD19 LiDAR (incl. motor) | 0.25 A                 |
| Pixhawk (logic)          | 0.20 A                 |

Power the Jetson from the airframe's BEC (not USB from a laptop) once
the LiDAR + autopilot are both connected.
