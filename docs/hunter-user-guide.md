# Hunter Receiver

---

## Contents

1. [Before You Power On](#before-you-power-on)
2. [Setup](#setup)
3. [Frequency Variants](#frequency-variants)
4. [Configuring with the UART Shell](#configuring-with-the-uart-shell)
5. [Reading the Raw UART Stream](#reading-the-raw-uart-stream)
6. [Troubleshooting](#troubleshooting)

---

## Before You Power On

**Always attach the antenna before plugging in.** The SMA antenna connector is on the edge of the board. Hand-tighten until snug. Operating without a proper antenna attached can damage the radio.

---

## Setup

1. Attach the antenna to the SMA connector on Hunter.
2. Plug Hunter into your computer using the USB-C cable.
3. Open **Dispatch** on your computer or a terminal emulator of your choice if you want to read the raw UART stream (see below).
4. Click the refresh icon next to the **Port** dropdown and select the Hunter's port from the list.
5. Click **Connect**.

Hunter will immediately begin listening for Outlaw packets. No further configuration is needed.

> **Not sure which port to select?** On Windows it will appear as a `COM` port (e.g. `COM4`). On macOS it will appear as `/dev/cu.usbserial-...`. On Linux, `/dev/ttyUSB0` or similar. If multiple ports appear, try unplugging Hunter, noting which ports are listed, then plugging it back in — the new entry is Hunter.

---

## Frequency Variants

Hunter must be on the same frequency variant as the Outlaw trackers you are receiving from. There are two variants:

| Variant | Frequency   |
|---|-------------|
| Standard | 903.000 MHz |
| Licensed | 435.000 MHz |

Hunter stores its receive frequency in persistent flash and can be reconfigured over the UART shell. The build variant still sets the allowed band:

- Standard builds can be tuned within `902.0` to `928.0` MHz
- Licensed builds can be tuned within `410.0` to `450.0` MHz

Hunter and the Outlaw trackers you want to receive still need to be on the same frequency.

---

## Configuring with the UART Shell

Hunter exposes a text shell over its USB serial port, which you can use to change the receive frequency without reflashing the firmware.

**What you need:**

- A computer with a terminal application such as PuTTY, Screen, minicom, or similar
- A USB-C cable connected to Hunter

**Connecting:**

Open the Hunter serial port at **115200 baud**. On connection you should see:

```
uart:~$
```

If no prompt appears immediately, press Enter once.

**Available command:**

Set the receive frequency:

```
uart:~$ config freq 903.125000
```

On licensed builds, pick a frequency within the licensed band instead, for example:

```
uart:~$ config freq 435.000000
```

The frequency is saved automatically and persists across reboots. Changes take effect after a reset or power cycle.

---

## Reading the Raw UART Stream

If you want to monitor Deputy directly without Dispatch, you can open Deputy's serial port in any terminal application at **9600 baud** and read the output as plain text.

Each time Deputy receives a packet from an Outlaw tracker it prints a short block of lines. There are two formats depending on whether the tracker has a GPS fix.

**Standard packet with a fix (unlicensed build):**
```
Node 1: (12 bytes | -87 dBm | 9 dB):
	Latitude: 43.084834
	Longitude: -77.680578
	Satellites count: 8
	Fix status: FIX
```

**Standard packet with a fix (licensed build):**
```
KD2YIE-1: (18 bytes | -87 dBm | 9 dB):
	Callsign: KD2YIE
	Latitude: 43.084834
	Longitude: -77.680578w
	Satellites count: 8
	Fix status: FIX
```

**No-fix packet:**
```
Node 1: (6 bytes | -91 dBm | 7 dB):
	No fix acquired
```

**Reading each field:**

| Field                    | What it tells you |
|--------------------------|---|
| `Node 1` / `KD2YIE-1`    | Which tracker this packet is from — node ID, with callsign prepended on licensed builds |
| `12 bytes` / `18 bytes`  | Packet size — 12 bytes for unlicensed, 18 bytes for licensed |
| `-87 dBm`                | Signal strength at the receiver. Less negative is better. Anything better than −110 dBm is a solid link |
| `9 dB`                   | Signal-to-noise ratio. Above 0 dB means a decodable signal; higher is better |
| `Latitude` / `Longitude` | GPS position in decimal degrees. Negative longitude is West, negative latitude is South |
| `Satellites count`       | How many satellites the tracker is currently using |
| `Fix status`             | `FIX` = good lock, `DIFF` = differential fix, `EST` = estimated, `NOFIX` = no lock yet |

Each packet is a self-contained block. If you see a `Node` or callsign header line followed by `No fix acquired`, the tracker is alive and transmitting but has not yet locked onto satellites.

---

## Dispatch Integration
Dispatch is a GUI application that interfaces with Hunter over serial to display tracker positions on a map in real time. It also logs every packet received for later export and analysis.
You can reference the [Dispatch user guide](https://github.com/AarC10/Dispatch-GSW/blob/main/docs/GUIDE.md) for more information on using Dispatch.

---

## Troubleshooting

**No packets appearing, Outlaw is powered on**
- Confirm you can see UART output from Hunter in a terminal application. You can press the reset button on Hunter to see the boot messages.
- Check that the antenna is firmly attached to Hunter.
- Check that the Outlaw's antenna is also attached.
- Confirm both units are the same frequency variant (Standard or Licensed).
- Make sure the Outlaw has had enough time outdoors to acquire a GPS fix — this can take a few minutes from a cold start.

**Hunter not showing up as a serial port**
- Try a different USB-C cable. Some cables are charge-only and carry no data.
- On Windows, a driver for the onboard USB-UART chip may be required. Download and install the **CH340 driver** from the chip manufacturer, then replug Hunter.

**"Failed to connect" error in Dispatch**
- Check that no other application (such as a terminal emulator) has the port open.
- Try clicking the port refresh icon and re-selecting the port before clicking Connect.

---
