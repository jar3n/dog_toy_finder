#!/usr/bin/env python3
"""Receive UDP log messages from the Pico firmware."""

import argparse
import socket


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Listen for UDP logs from the dog toy finder Pico."
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="local address to bind (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5005,
        help="UDP port matching WIFI_LOG_TARGET_PORT (default: 5005)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))

    print(
        f"Listening for Pico logs on udp://{args.bind}:{args.port}",
        flush=True,
    )

    try:
        while True:
            data, address = sock.recvfrom(65535)
            text = data.decode("utf-8", errors="replace").rstrip("\r\n")
            print(f"{address[0]}: {text}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
