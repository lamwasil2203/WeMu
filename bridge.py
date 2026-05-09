"""
bridge.py — pipe ranging output through this to forward distance to ESP32

Usage:
    python run_fira_twr.py -p /dev/cu.usbmodemXXX -t -1 | python bridge.py --esp /dev/cu.usbserialXXX

If you don't know the ESP32 port yet, run without --esp to just print parsed distances:
    python run_fira_twr.py -p /dev/cu.usbmodemXXX -t -1 | python bridge.py
"""

import sys
import re
import argparse
import time

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--esp", type=str, default=None,
                        help="Serial port of ESP32 (e.g. /dev/cu.usbserial-XXXX)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    esp = None
    if args.esp:
        import serial
        esp = serial.Serial(args.esp, args.baud, timeout=1)
        print(f"[bridge] Connected to ESP32 on {args.esp}", file=sys.stderr)
    else:
        print("[bridge] No ESP32 port specified — printing distances only", file=sys.stderr)

    current_status = "Ok"

    for line in sys.stdin:
        sys.stdout.write(line)  # pass through so terminal still shows ranging output
        sys.stdout.flush()

        # Track status to skip bad measurements
        status_match = re.search(r'status:\s+(\w+)', line)
        if status_match:
            current_status = status_match.group(1)

        # Parse distance
        dist_match = re.search(r'distance:\s+([\d.]+)\s+cm', line)
        if dist_match and current_status == "Ok":
            dist_cm = float(dist_match.group(1))

            # Clamp to a sane range (CDK can report garbage at extremes)
            dist_cm = max(10.0, min(dist_cm, 500.0))

            msg = f"DIST:{dist_cm:.1f}\n"
            print(f"[bridge] → ESP32: {msg.strip()}", file=sys.stderr)

            if esp:
                try:
                    esp.write(msg.encode())
                except Exception as e:
                    print(f"[bridge] ESP32 write error: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()