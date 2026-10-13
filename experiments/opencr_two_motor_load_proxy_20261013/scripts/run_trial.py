#!/usr/bin/env python3
"""Raspberry Pi USB-serial trial runner for OpenCR two-motor load-PROXY experiment.

No supply/bus current is measured. The OpenCR reports per-motor current telemetry.
Install dependency: python3 -m pip install pyserial (inside a virtualenv).
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("Missing pyserial. Activate project venv, then: pip install pyserial")

FIELDS = [
    "motor_ms", "mode", "steer_current_mA", "drive_current_mA",
    "steer_voltage_V", "drive_voltage_V", "steer_position_ticks",
    "drive_velocity_raw", "steer_temp_C", "drive_temp_C", "steer_pwm_raw", "drive_pwm_raw",
]
MODES = ("IDLE", "STEER", "DRIVE", "BOTH")


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def new_path(root, name):
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = root / f"{stamp}_{name.lower()}"
    suffix = 0
    while path.exists():
        suffix += 1
        path = root / f"{stamp}_{name.lower()}_{suffix}"
    path.mkdir(parents=True)
    return path


def git_info():
    def git(*args):
        try:
            return subprocess.check_output(["git", *args], stderr=subprocess.DEVNULL, text=True).strip()
        except (OSError, subprocess.CalledProcessError):
            return None
    return {"commit": git("rev-parse", "HEAD"), "branch": git("branch", "--show-current"),
            "remote": git("remote", "get-url", "origin"), "dirty": bool(git("status", "--porcelain"))}


def main():
    parser = argparse.ArgumentParser(description="One safe, timed, single-mode trial")
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--port", required=True, help="OpenCR /dev/serial/by-id/... stable symlink")
    parser.add_argument("--out", default="results/raw", help="Directory for a NEW trial subdirectory")
    parser.add_argument("--note", default="", help="Short physical setup/load note")
    args = parser.parse_args()
    if not 2 <= args.seconds <= 30:
        parser.error("--seconds must be 2..30; use several separate runs instead of a long unattended test")
    out = new_path(Path(args.out).expanduser().resolve(), args.mode)
    meta = {
        "mode": args.mode, "planned_seconds": args.seconds, "port": args.port,
        "note": args.note, "start_utc": utc_now(), "hostname": platform.node(),
        "os": platform.platform(), "python": sys.version.split()[0],
        "pyserial": getattr(serial, "VERSION", "unknown"), "git": git_info(),
        "measurement_scope": "motor-present-current proxies; NOT OpenCR or PSU input current",
    }
    (out / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")
    print("Recording to:", out)
    started = False
    finished_safely = False
    samples = 0
    capture_active = False
    faults = []
    raw = (out / "serial_raw.txt").open("w", buffering=1)
    rows = (out / "telemetry.csv").open("w", newline="", buffering=1)
    csv_writer = csv.writer(rows)
    csv_writer.writerow(["host_utc", "host_mono_ns", *FIELDS])
    port = None

    def send(command):
        if port is None:
            raise RuntimeError("Serial port not open")
        port.write((command + "\n").encode("ascii"))
        port.flush()
        raw.write(f"# TX {utc_now()} {command}\n")

    def recv():
        nonlocal samples
        if port is None:
            return None
        buf = port.readline()
        if not buf:
            return None
        now = utc_now()
        mono = time.monotonic_ns()
        line = buf.decode("ascii", errors="replace").strip()
        raw.write(f"{now} {line}\n")
        if line.startswith("FAULT,"):
            faults.append(line)
            print("Firmware fault:", line, file=sys.stderr)
            raise RuntimeError(line)
        if line.startswith("TEL,") and capture_active:
            fields = line.split(",")[1:]
            if len(fields) != len(FIELDS):
                raw.write("# WARN malformed telemetry line\n")
            else:
                csv_writer.writerow([now, mono, *fields])
                samples += 1
        return line

    def wait_for(match, timeout):
        deadline = time.monotonic() + timeout
        last_hello = 0.0
        while time.monotonic() < deadline:
            if match == "READY" and time.monotonic() - last_hello > 1:
                send("HELLO")
                last_hello = time.monotonic()
            line = recv()
            if line is not None and line.startswith(match):
                return line
        raise RuntimeError("Timed out waiting for: " + match)

    try:
        port = serial.Serial(args.port, 115200, timeout=0.06, write_timeout=0.5)
        time.sleep(1.2)  # Opening USB may reset some OpenCR sketches.
        ready = wait_for("READY", 8)
        if "two-motor-telemetry-v1" not in ready:
            raise RuntimeError("Unexpected OpenCR firmware: " + ready)
        status = wait_for("STATUS,", 3)  # READY/STATUS emitted by HELLO
        if not status.startswith("STATUS,MOTORS_OK,DISARMED"):
            raise RuntimeError("Refusing to arm: " + status)
        print("Firmware identified; both DYNAMIXEL motors detected; beginning", args.mode)
        send("ARM")
        wait_for("ACK,ARMED", 4)
        started = True
        send("RUN " + args.mode)
        wait_for("ACK,RUN," + args.mode, 4)
        capture_active = True
        end = time.monotonic() + args.seconds
        last_hb = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_hb >= 0.2:
                send("HB")
                last_hb = now
            recv()
        capture_active = False
        send("DISARM")
        wait_for("ACK,DISARMED", 4)
        finished_safely = True
        print(f"Trial complete: {samples} telemetry samples; DISARM acknowledged.")
    except (Exception, KeyboardInterrupt) as error:
        raw.write(f"# ERROR {utc_now()} {repr(error)}\n")
        print("Trial interrupted; attempting DISARM:", error, file=sys.stderr)
        raise
    finally:
        try:
            if port and port.is_open:
                send("DISARM")
                time.sleep(0.1)
                port.close()
        except Exception as err:
            raw.write(f"# WARN failed final DISARM: {err!r}\n")
        raw.close()
        rows.close()
        meta["end_utc"] = utc_now()
        meta["sample_count"] = samples
        meta["disarm_acknowledged"] = finished_safely
        meta["faults"] = faults
        (out / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")
        if started and not finished_safely:
            print("CHECK HARDWARE: confirm motors stopped; switch OFF motor power if needed.", file=sys.stderr)

if __name__ == "__main__":
    main()
