"""
bridge.py — laptop bridge for testing before relay ESP32s are wired

Reads ranging output from the two CONTROLLER CDKs and forwards
tagged distances to the master ESP32 over USB serial.

Usage (4 terminals):

  T1 — pair 1 controller (ch 9, default):
    python run_fira_twr.py -p /dev/cu.usbmodemXXX -t -1 | python bridge_dual.py --pair 1 --esp /dev/cu.usbserialXXX

  T2 — pair 1 controlee:
    python run_fira_twr.py -p /dev/cu.usbmodemYYY --controlee -t -1

  T3 — pair 2 controller (ch 5):
    python run_fira_twr.py -p /dev/cu.usbmodemAAA -t -1 -c 5 | python bridge_dual.py --pair 2 --esp /dev/cu.usbserialXXX

  T4 — pair 2 controlee (ch 5):
    python run_fira_twr.py -p /dev/cu.usbmodemBBB --controlee -t -1 -c 5

Both bridge instances write to the SAME master ESP32 serial port.
"""

import sys
import re
import argparse
import serial

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pair", type=int, required=True, choices=[1, 2])
    parser.add_argument("--esp",  type=str, required=True,
                        help="Master ESP32 port e.g. /dev/cu.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    esp = serial.Serial(args.esp, args.baud, timeout=1)
    print(f"[bridge pair {args.pair}] connected on {args.esp}", file=sys.stderr)

    current_status = "Ok"

    for line in sys.stdin:
        sys.stdout.write(line)
        sys.stdout.flush()

        status_match = re.search(r'status:\s+(\w+)', line)
        if status_match:
            current_status = status_match.group(1)

        dist_match = re.search(r'distance:\s+([\d.]+)\s+cm', line)
        if dist_match and current_status == "Ok":
            dist_cm = float(dist_match.group(1))
            dist_cm = max(10.0, min(dist_cm, 400.0))
            msg = f"DIST{args.pair}:{dist_cm:.1f}\n"
            print(f"[bridge pair {args.pair}] → {msg.strip()}", file=sys.stderr)
            try:
                esp.write(msg.encode())
            except Exception as e:
                print(f"[bridge pair {args.pair}] write error: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()
