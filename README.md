# WeMu

UWB-based human theremin. Two pairs of dancers produce sound based on their proximity.

## Repo structure

```
WeMu/
├── platformio.ini          multi-environment build
├── include/
│   └── shared.h            MAC address, message struct, pin constants
├── src/
│   ├── relay/
│   │   └── main.cpp        flash to all 4 CDK-paired ESP32s
│   └── master/
│       └── main.cpp        flash to the audio master ESP32
├── bridge_dual.py          laptop bridge (for testing without relay ESP32s)
└── README.md
```

## Hardware per dancer unit

- 1× DWM3001CDK (UWB ranging)
- 1× TTGO T1 / ESP32 (relay — power + data forwarding)

## Wiring (CDK Pi header → relay ESP32)

| CDK Pi Header | Signal | ESP32 Pin |
|---|---|---|
| Pin 4 | 5V | VIN |
| Pin 6 | GND | GND |
| Pin 8 | TX | GPIO16 (RX2) |
| Pin 10 | RX | GPIO17 (TX2) |

## Pair ID selection

Wire GPIO34 to GND on the two pair-2 ESP32s.
Leave GPIO34 floating on the two pair-1 ESP32s.

## Flash order

### 1. Flash master first

```bash
pio run -e master -t upload
```

Open serial monitor, copy the printed MAC:
```
[master] MAC address: AA:BB:CC:DD:EE:FF
```

### 2. Update shared.h

```cpp
static uint8_t MASTER_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

### 3. Flash all 4 relay ESP32s

```bash
pio run -e relay -t upload
```

Remember: pair-2 units need GPIO34 bridged to GND before flashing.

## CDK firmware

Flash both CDKs in each pair with the UCI firmware from the Qorvo SDK:
- `DWM3001CDK-DW3_QM33_SDK_UCI-FreeRTOS.hex`

Pair 1 runs on **channel 9** (default).
Pair 2 runs on **channel 5** — set via the ranging script `-c 5` flag.

## Testing with laptop (before relay ESP32s are wired)

Run 4 terminals:

```bash
# T1 — pair 1 controller
python run_fira_twr.py -p /dev/cu.usbmodemXXX -t -1 | python bridge_dual.py --pair 1 --esp /dev/cu.usbserialXXX

# T2 — pair 1 controlee
python run_fira_twr.py -p /dev/cu.usbmodemYYY --controlee -t -1

# T3 — pair 2 controller (channel 5)
python run_fira_twr.py -p /dev/cu.usbmodemAAA -t -1 -c 5 | python bridge_dual.py --pair 2 --esp /dev/cu.usbserialXXX

# T4 — pair 2 controlee (channel 5)
python run_fira_twr.py -p /dev/cu.usbmodemBBB --controlee -t -1 -c 5
```

## Sound design

| Distance | Pair 1 (pitch) | Pair 2 (rhythm) |
|---|---|---|
| < 60 cm | Dissonant cluster, gritty | Fast pulse, heavy sub bass |
| 60–200 cm | Fifth/fourth intervals, warm | Moderate pulse, texture |
| > 200 cm | Pure octave drone | Slow breath, fades out |
