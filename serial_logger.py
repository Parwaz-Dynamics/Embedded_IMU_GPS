#!/usr/bin/env python3
"""
Serial logger for IMU / GPS / EKF / Filter data.

This script listens on a serial port, splits incoming messages by prefix,
and writes each message type into its own CSV file.

Supported message types:
- IMU, ...
- GPS, ...
- EKF, ...
- Res, ... (stored in Filter.csv)

Usage example:
    python serial_logger.py --port COM10 --baud 115200 --out-dir logs
"""

import argparse
import csv
import os
import sys
import time
from pathlib import Path

try:
    import serial
except Exception as exc:
    print(f"Error: pyserial is required. Install with: pip install pyserial\n{exc}")
    sys.exit(1)


HEADER_MAP = {
    "IMU": [
        "time",
        "ax",
        "ay",
        "az",
        "wx",
        "wy",
        "wz",
    ],
    "GPS": [
        "time",
        "latitude",
        "longitude",
        "altitude",
        "vn",
        "ve",
        "hdop",
        "satellites",
    ],
    "EKF": [
        "time",
        "latitude",
        "longitude",
        "altitude",
        "vn",
        "ve",
        "vd",
        "roll",
        "pitch",
        "yaw",
    ],
    "Filter": [
        "t_s",
        "px",
        "py",
        "pz",
        "vn",
        "ve",
        "vd",
        "qw",
        "qx",
        "qy",
        "qz",
        "bgx",
        "bgy",
        "bgz",
        "bax",
        "bay",
        "baz",
        "P_px",
        "P_py",
        "P_pz",
        "P_vn",
        "P_ve",
        "P_vd",
        "P_rn",
        "P_re",
        "P_rd",
        "P_bgx",
        "P_bgy",
        "P_bgz",
        "P_bax",
        "P_bay",
        "P_baz",
        "innov_pn",
        "innov_pe",
        "innov_pd",
        "innov_vn",
        "innov_ve",
        "innov_vd",
        "S_pn",
        "S_pe",
        "S_pd",
        "S_vn",
        "S_ve",
        "S_vd",
        "rejected",
    ],
}


def parse_args():
    parser = argparse.ArgumentParser(description="Log serial data to CSV files.")
    parser.add_argument("--port", default="COM20", help="Serial port (example: COM20 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds")
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="Maximum run time in seconds (0 = run until stopped manually)",
    )
    parser.add_argument(
        "--out-dir",
        default=".",
        help="Directory where IMU.csv, GPS.csv, EKF.csv, and Filter.csv will be created",
    )
    return parser.parse_args()


def open_csvs(out_dir: Path):
    files = {}
    for name, headers in HEADER_MAP.items():
        csv_path = out_dir / f"{name}.csv"
        exists = csv_path.exists()
        handle = csv_path.open("a", newline="", encoding="utf-8")
        writer = csv.writer(handle)

        if not exists:
            writer.writerow(headers)

        files[name] = {
            "path": csv_path,
            "handle": handle,
            "writer": writer,
        }

    return files


def write_row(csv_info, data):
    csv_info["writer"].writerow(data)
    csv_info["handle"].flush()


def parse_message(line: str):
    stripped = line.strip()
    if not stripped:
        return None

    parts = stripped.split(",")
    if len(parts) < 2:
        return None

    msg_type = parts[0].strip()

    if msg_type == "IMU":
        return "IMU", parts[1:8]

    if msg_type == "GPS":
        # GPS,time,lat,lon,alt,vn,ve,hdop,satellites
        return "GPS", parts[1:9]

    if msg_type == "EKF":
        # EKF,time,lat,lon,alt,vn,ve,vd,roll,pitch,yaw
        return "EKF", parts[1:11]

    if msg_type == "Res":
        # Filter contains the full Res line, with all fields after Res,
        return "Filter", parts[1:]

    return None


def main():
    args = parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for name in HEADER_MAP:
        csv_path = out_dir / f"{name}.csv"
        if csv_path.exists():
            csv_path.unlink()

    csv_files = open_csvs(out_dir)

    stop_time = None if args.duration <= 0 else time.monotonic() + args.duration

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
        print(f"Connected to {args.port} at {args.baud} baud")
    except Exception as exc:
        print(f"Failed to open serial port {args.port}: {exc}")
        return 1

    try:
        while True:
            if stop_time is not None and time.monotonic() >= stop_time:
                print(f"\nStopped after {args.duration} seconds.")
                break

            try:
                raw_line = ser.readline()
            except serial.SerialException as exc:
                print(f"Serial read error: {exc}")
                break

            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace")
            parsed = parse_message(line)
            if parsed is None:
                continue

            msg_type, data = parsed
            if msg_type in csv_files:
                write_row(csv_files[msg_type], data)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

        for info in csv_files.values():
            try:
                info["handle"].close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
