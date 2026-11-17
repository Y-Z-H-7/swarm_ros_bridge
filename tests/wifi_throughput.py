#!/usr/bin/env python3
"""Short, in-memory TCP throughput measurement between nuc2 and GCS."""
import argparse
import socket
import time

CHUNK = bytes(1024 * 1024)


def report(role, direction, count, elapsed):
    print(f"{role} {direction}: {count / 1e6:.1f} MB in {elapsed:.2f} s = "
          f"{count * 8 / elapsed / 1e6:.1f} Mbit/s", flush=True)


def handle(conn, address):
    with conn:
        conn.settimeout(30)
        header = bytearray()
        while not header.endswith(b"\n"):
            piece = conn.recv(1)
            if not piece:
                return
            header.extend(piece)
        direction, seconds_text = header.decode("ascii").strip().split()
        seconds = float(seconds_text)
        conn.sendall(b"R")
        start = time.monotonic()
        count = 0
        if direction == "upload":
            while True:
                piece = conn.recv(1024 * 1024)
                if not piece:
                    break
                count += len(piece)
        elif direction == "download":
            end = start + seconds
            while time.monotonic() < end:
                conn.sendall(CHUNK)
                count += len(CHUNK)
            conn.shutdown(socket.SHUT_WR)
        else:
            raise ValueError(direction)
        report("server", direction, count, time.monotonic() - start)


def server(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port))
        listener.listen(1)
        listener.settimeout(90)
        print(f"listening on {host}:{port}", flush=True)
        for _ in range(2):
            conn, address = listener.accept()
            handle(conn, address)


def client(host, port, direction, seconds):
    with socket.create_connection((host, port), timeout=10) as conn:
        conn.settimeout(30)
        conn.sendall(f"{direction} {seconds}\n".encode("ascii"))
        if conn.recv(1) != b"R":
            raise RuntimeError("server did not acknowledge")
        start = time.monotonic()
        count = 0
        if direction == "upload":
            end = start + seconds
            while time.monotonic() < end:
                conn.sendall(CHUNK)
                count += len(CHUNK)
            conn.shutdown(socket.SHUT_WR)
        else:
            while True:
                piece = conn.recv(1024 * 1024)
                if not piece:
                    break
                count += len(piece)
        report("client", direction, count, time.monotonic() - start)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("server", "upload", "download"))
    parser.add_argument("--host", default="192.168.0.137")
    parser.add_argument("--port", type=int, default=55150)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()
    if args.mode == "server":
        server(args.host, args.port)
    else:
        client(args.host, args.port, args.mode, args.seconds)


if __name__ == "__main__":
    main()
