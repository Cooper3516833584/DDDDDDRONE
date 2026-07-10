"""Minimal serial terminal for HC-14 link checks.

This script talks only to the local USB-TTL serial port. It does not import or
control the flight controller.
"""

import argparse
import sys
import threading
import time
from datetime import datetime
from typing import Optional

import serial
from serial.tools import list_ports


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device:12} {port.description} {port.hwid}")


def format_bytes(data: bytes, hex_only: bool, encoding: str) -> str:
    hex_text = " ".join(f"{byte:02X}" for byte in data)
    if hex_only:
        return hex_text
    try:
        text = data.decode(encoding, errors="replace")
    except LookupError:
        text = data.decode("utf-8", errors="replace")
    text = text.replace("\r", "\\r").replace("\n", "\\n")
    return f"{text}    [hex: {hex_text}]"


def open_hc14_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.1
    ser.write_timeout = 1
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.rts = False
    ser.dtr = False
    ser.open()
    ser.setRTS(False)
    ser.setDTR(False)
    return ser


def reader_worker(ser: serial.Serial, stop_event: threading.Event, hex_only: bool, encoding: str) -> None:
    while not stop_event.is_set():
        try:
            data = ser.read(max(1, ser.in_waiting))
        except serial.SerialException as exc:
            print(f"\n[ERR] serial read failed: {exc}")
            stop_event.set()
            return
        if data:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"\n[RX {ts} {len(data)}B] {format_bytes(data, hex_only, encoding)}")
            print("> ", end="", flush=True)


def periodic_sender(
    ser: serial.Serial,
    stop_event: threading.Event,
    message: bytes,
    period: Optional[float],
) -> None:
    if period is None:
        return
    while not stop_event.wait(period):
        try:
            ser.write(message)
            ser.flush()
            print(f"\n[TX periodic {len(message)}B] {message!r}")
            print("> ", end="", flush=True)
        except serial.SerialException as exc:
            print(f"\n[ERR] serial write failed: {exc}")
            stop_event.set()
            return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HC-14 serial terminal/link checker")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", help="serial port, for example COM7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="baud rate, default: 9600")
    parser.add_argument("--encoding", default="utf-8", help="text encoding for display/input")
    parser.add_argument("--hex", action="store_true", help="display received data as hex only")
    parser.add_argument("--no-stdin", action="store_true", help="do not read keyboard input")
    parser.add_argument("--send-every", type=float, help="send --message every N seconds")
    parser.add_argument("--message", default="HC14_PING\n", help="message used by --send-every")
    parser.add_argument(
        "--raw-input",
        action="store_true",
        help="do not append a newline to keyboard input",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        list_serial_ports()
        print("\nPass --port explicitly before opening a serial device.")
        return 2

    message = args.message.encode(args.encoding, errors="replace")
    stop_event = threading.Event()

    try:
        with open_hc14_serial(args.port, args.baud) as ser:
            print(f"Opened {args.port} at {args.baud} baud.")
            print("HC-14 control lines: DTR=False, RTS=False.")
            print("Type text and press Enter to send. Press Ctrl+C to exit.")
            reader = threading.Thread(
                target=reader_worker,
                args=(ser, stop_event, args.hex, args.encoding),
                daemon=True,
            )
            sender = threading.Thread(
                target=periodic_sender,
                args=(ser, stop_event, message, args.send_every),
                daemon=True,
            )
            reader.start()
            sender.start()

            if args.no_stdin:
                while not stop_event.wait(0.2):
                    pass
            else:
                while not stop_event.is_set():
                    try:
                        line = input("> ")
                    except EOFError:
                        break
                    payload = line if args.raw_input else line + "\n"
                    data = payload.encode(args.encoding, errors="replace")
                    ser.write(data)
                    ser.flush()
                    print(f"[TX {len(data)}B] {data!r}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    finally:
        stop_event.set()
        time.sleep(0.2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
