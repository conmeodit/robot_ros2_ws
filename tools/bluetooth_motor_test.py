#!/usr/bin/env python3
"""Small Bluetooth serial motor test for the Arduino Mega base firmware.

The firmware listens on Serial2/HC-05 at 9600 baud and accepts either single
manual commands (F/B/L/R/S) or ROS-like commands (CMD_VEL,v,w).
"""

import argparse
import select
import sys
import termios
import threading
import time
import tty

try:
    import serial
except ImportError:
    serial = None


MANUAL_COMMANDS = {
    "f": "F",
    "forward": "F",
    "b": "B",
    "back": "B",
    "l": "L",
    "left": "L",
    "r": "R",
    "right": "R",
    "s": "S",
    "stop": "S",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test Arduino Mega motor commands through Bluetooth serial."
    )
    parser.add_argument("--port", default="/dev/rfcomm0", help="Bluetooth serial port")
    parser.add_argument("--baud", type=int, default=9600, help="Bluetooth baudrate")
    parser.add_argument("--rate", type=float, default=10.0, help="command repeat rate in Hz")
    parser.add_argument("--duration", type=float, default=1.0, help="movement duration in seconds")
    parser.add_argument(
        "--cmd",
        choices=sorted(MANUAL_COMMANDS),
        help="manual command: forward/back/left/right/stop or f/b/l/r/s",
    )
    parser.add_argument(
        "--cmd-vel",
        nargs=2,
        type=float,
        metavar=("LINEAR_MPS", "ANGULAR_RADPS"),
        help="send repeated CMD_VEL,linear,angular",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="keyboard mode: w/a/s/d move, x or space stop, q quit",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="short F/B/L/R demo sequence; use only with wheels off the ground first",
    )
    parser.add_argument("--no-telemetry", action="store_true", help="do not print STAT lines")
    return parser.parse_args()


def open_serial(port, baud):
    if serial is None:
        raise SystemExit("pyserial is missing. Install with: python3 -m pip install pyserial")
    return serial.Serial(port=port, baudrate=baud, timeout=0.02, write_timeout=0.2)


def write_line(ser, line):
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def stop_motors(ser):
    for _ in range(3):
        write_line(ser, "S")
        write_line(ser, "CMD_VEL,0.0000,0.0000")
        time.sleep(0.05)


def telemetry_reader(ser, stop_event):
    buffer = ""
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
        except Exception as exc:
            print(f"\n[read error] {exc}", file=sys.stderr)
            return
        if not data:
            continue
        buffer += data.decode("ascii", errors="replace")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line:
                print(f"[bt] {line}")


def repeat_command(ser, line, duration, rate):
    period = 1.0 / max(rate, 1.0)
    deadline = time.monotonic() + max(duration, 0.0)
    while time.monotonic() < deadline:
        write_line(ser, line)
        time.sleep(period)


def enable_drive(ser):
    write_line(ser, "ESTOP,0")
    time.sleep(0.05)
    write_line(ser, "MOTOR,1")
    time.sleep(0.05)


def run_demo(ser, duration, rate):
    for label, line in (
        ("forward", "F"),
        ("back", "B"),
        ("left", "L"),
        ("right", "R"),
    ):
        print(f"[send] {label}: {line}")
        repeat_command(ser, line, duration, rate)
        stop_motors(ser)
        time.sleep(0.4)


def interactive(ser, rate):
    print("Interactive Bluetooth motor test")
    print("  w: forward, a: left, d: right, s: back, x/space: stop, q: quit")
    print("Commands repeat while you keep pressing keys; firmware stops after 500 ms timeout.")
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_line = "S"
    last_send = 0.0
    period = 1.0 / max(rate, 1.0)
    key_to_line = {
        "w": "F",
        "a": "L",
        "d": "R",
        "s": "B",
        "x": "S",
        " ": "S",
    }
    try:
        tty.setcbreak(fd)
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == "q":
                    break
                if key in key_to_line:
                    last_line = key_to_line[key]
                    print(f"\n[send] {last_line}")

            now = time.monotonic()
            if now - last_send >= period:
                write_line(ser, last_line)
                last_send = now
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        stop_motors(ser)


def main():
    args = parse_args()
    if not (args.interactive or args.demo or args.cmd or args.cmd_vel):
        raise SystemExit("Choose --interactive, --demo, --cmd, or --cmd-vel.")

    with open_serial(args.port, args.baud) as ser:
        print(f"Connected {args.port} at {args.baud} baud")
        stop_event = threading.Event()
        reader = None
        if not args.no_telemetry:
            reader = threading.Thread(target=telemetry_reader, args=(ser, stop_event), daemon=True)
            reader.start()

        try:
            enable_drive(ser)
            if args.interactive:
                interactive(ser, args.rate)
            elif args.demo:
                run_demo(ser, args.duration, args.rate)
            elif args.cmd:
                line = MANUAL_COMMANDS[args.cmd]
                print(f"[send] {line} for {args.duration:.2f}s")
                repeat_command(ser, line, args.duration, args.rate)
            elif args.cmd_vel:
                linear, angular = args.cmd_vel
                line = f"CMD_VEL,{linear:.4f},{angular:.4f}"
                print(f"[send] {line} for {args.duration:.2f}s")
                repeat_command(ser, line, args.duration, args.rate)
        finally:
            stop_motors(ser)
            stop_event.set()
            if reader is not None:
                reader.join(timeout=0.2)
            print("Stopped motors")


if __name__ == "__main__":
    main()
