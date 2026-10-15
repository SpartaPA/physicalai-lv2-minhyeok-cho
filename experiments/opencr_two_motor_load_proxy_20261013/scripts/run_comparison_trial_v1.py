import argparse
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path

import serial

parser = argparse.ArgumentParser()
parser.add_argument("action", choices=["INIT", "IDLE", "STEER", "DRIVE", "BOTH"])
parser.add_argument("--repeat", type=int, default=1)
args = parser.parse_args()
if args.repeat < 1:
    parser.error("--repeat must be positive")

duration = 12.0
root = Path(os.environ["EXP"])
folder = root / "results" / "comparison"
folder.mkdir(parents=True, exist_ok=True)
stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
stem = f"{args.action.lower()}_r{args.repeat}_{stamp}"
log_path = folder / f"{stem}.txt"
csv_path = folder / f"{stem}.csv"

expected_ready = (
    "READY,opencr-two-motor-telemetry-v3,"
    "STEER_ID=11,DRIVE_ID=1,INACTIVE_IDS=12|2"
)
guard_line = "GUARD,INACTIVE_12=OFF,INACTIVE_2=OFF"
error_prefixes = (
    "FAULT,", "ERR,", "READ_ERROR,", "CHECK_FAILED,", "STOP_UNVERIFIED,"
)

success = False
stop_ok = False
attempted = False
collecting = False
samples = []
last_tel = 0.0
last_guard = 0.0
last_device_ms = None
next_hb = 0.0

with log_path.open("x", buffering=1) as log:
    def record(message):
        print(message, flush=True)
        log.write(f"{datetime.now().isoformat()} {message}\n")

    record(f"HOST: action={args.action}, repeat={args.repeat}, duration={duration}")
    record("HOST: Current telemetry is not external supply-current measurement.")

    with serial.Serial(
        os.environ["PORT"], 115200, timeout=0.02, write_timeout=1
    ) as port:
        pending = bytearray()

        def send(command):
            record(f"HOST: {command}")
            port.write((command + "\n").encode())
            port.flush()

        def receive(seconds, heartbeat=False, until=None, strict=True):
            global last_tel, last_guard, last_device_ms, next_hb
            lines = []
            deadline = time.monotonic() + seconds

            while time.monotonic() < deadline:
                now = time.monotonic()

                if heartbeat:
                    if now - last_tel > 0.8:
                        raise RuntimeError("Telemetry missing for over 0.8 seconds.")
                    if now - last_guard > 0.8:
                        raise RuntimeError("Inactive-motor guard missing for over 0.8 seconds.")
                    if now >= next_hb:
                        send("HB")
                        next_hb = now + 0.2

                pending.extend(port.read(port.in_waiting or 1))
                while b"\n" in pending:
                    raw, _, remainder = pending.partition(b"\n")
                    pending[:] = remainder
                    line = raw.rstrip(b"\r").decode(errors="replace")
                    record(line)
                    lines.append(line)

                    if strict and line.startswith(error_prefixes):
                        raise RuntimeError(line)

                    if line == guard_line:
                        last_guard = time.monotonic()

                    if line.startswith("TEL,"):
                        fields = line.split(",")
                        try:
                            if len(fields) != 13:
                                raise ValueError("Wrong field count")
                            device_ms = int(fields[1])
                            values = [float(v) for v in fields[3:]]
                            if not all(math.isfinite(v) for v in values):
                                raise ValueError("Non-finite value")
                            if fields[2] not in ("IDLE", "STEER", "DRIVE", "BOTH"):
                                raise ValueError("Unknown mode")
                        except ValueError as error:
                            if strict:
                                raise RuntimeError(f"Invalid telemetry: {line}") from error
                            continue

                        last_tel = time.monotonic()
                        if collecting:
                            if fields[2] != args.action:
                                raise RuntimeError("Unexpected mode during measurement.")
                            if last_device_ms is not None and device_ms <= last_device_ms:
                                raise RuntimeError("Non-increasing device timestamp.")
                            last_device_ms = device_ms
                            samples.append([datetime.now().isoformat()] + fields[1:])

                    if until is not None and line == until:
                        return lines

            if until is not None:
                raise RuntimeError(f"Missing response: {until}")
            return lines

        try:
            port.reset_input_buffer()
            send("")
            receive(1, strict=False)

            send("STATUS")
            initial = receive(1)
            if expected_ready not in initial:
                raise RuntimeError("Firmware or motor mapping mismatch.")

            if args.action == "INIT":
                if "STATUS,MOTORS_MISSING,DISARMED,IDLE" not in initial:
                    raise RuntimeError("INIT expects a fresh, disarmed boot.")
                attempted = True
                send("INIT")
                receive(5, until="INIT,OK")
                checked = receive(1)
                if guard_line not in checked:
                    raise RuntimeError("Inactive-motor guard not confirmed.")
                success = True
            else:
                if "STATUS,MOTORS_OK,DISARMED,IDLE" not in initial:
                    raise RuntimeError("Reset and initialize before this trial.")
                if guard_line not in initial:
                    raise RuntimeError("Inactive-motor guard not confirmed.")

                attempted = True
                last_tel = last_guard = time.monotonic()
                next_hb = time.monotonic()
                send("ARM")
                receive(1, heartbeat=True, until="ACK,ARMED")
                receive(1, heartbeat=True)

                send(f"RUN {args.action}")
                receive(1, heartbeat=True, until=f"ACK,RUN,{args.action}")

                # Discard one full steering period as a common settling window.
                receive(4, heartbeat=True)
                record("HOST: MEASUREMENT_BEGIN")
                collecting = True
                receive(duration, heartbeat=True)
                collecting = False
                record("HOST: MEASUREMENT_END")

                if len(samples) < 50:
                    raise RuntimeError("Too few samples for the 12-second window.")
                timestamps = [int(row[1]) for row in samples]
                if any(b - a > 600 for a, b in zip(timestamps, timestamps[1:])):
                    raise RuntimeError("Telemetry gap greater than 600 ms.")
                success = True

        except (Exception, KeyboardInterrupt) as error:
            record(f"HOST: FAILED_OR_INTERRUPTED: {error}")

        finally:
            collecting = False
            if attempted:
                try:
                    send("STOP")
                    stopped = receive(2, strict=False)
                    send("STATUS")
                    final = receive(1, strict=False)
                    stop_ok = (
                        "ACK,DISARMED" in stopped
                        and "STATUS,MOTORS_OK,DISARMED,IDLE" in final
                        and not any(
                            line.startswith(error_prefixes)
                            for line in stopped + final
                        )
                    )
                except Exception as error:
                    record(f"HOST: STOP_CHECK_FAILED: {error}")

    if samples:
        with csv_path.open("x", newline="") as output:
            writer = csv.writer(output)
            writer.writerow([
                "host_time", "device_ms", "mode",
                "steer_mA", "drive_mA", "steer_V", "drive_V",
                "steer_pos_raw", "drive_vel_raw",
                "steer_temp_C", "drive_temp_C",
                "steer_pwm_raw", "drive_pwm_raw",
            ])
            writer.writerows(samples)
        record(f"HOST: CSV={csv_path}")

    passed = success and stop_ok
    record(f"HOST: samples={len(samples)}, stop_verified={stop_ok}, passed={passed}")
    if not passed:
        record("HOST: Disconnect 12 V and inspect this log.")
    record(f"HOST: LOG={log_path}")

raise SystemExit(0 if passed else 1)
