#!/usr/bin/env python3
"""Exercise an ESP-SDR burst firmware over its serial protocol.

Standard library only (POSIX termios). Works against both the Rust port and
the original C firmware, which makes it a quick parity check:

    python3 tools/smoke_test.py /dev/cu.usbmodem1101

Exits non-zero on the first protocol error, CRC mismatch or bad reply.
"""

import math
import os
import select
import sys
import termios
import time
import zlib


class Port:
    def __init__(self, path, baud=2_000_000):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0  # iflag
        attrs[1] = 0  # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0  # lflag: raw
        speed = getattr(termios, f"B{baud}", termios.B115200)
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.buf = b""

    def write(self, line):
        os.write(self.fd, line.encode() + b"\n")

    def _fill(self, timeout):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            raise TimeoutError("no data from device")
        self.buf += os.read(self.fd, 65536)

    def readline(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buf:
            self._fill(max(0.0, deadline - time.monotonic()))
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode(errors="replace")

    def read(self, n, timeout=5.0):
        deadline = time.monotonic() + timeout
        while len(self.buf) < n:
            self._fill(max(0.0, deadline - time.monotonic()))
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def drain(self, quiet=0.3):
        while True:
            ready, _, _ = select.select([self.fd], [], [], quiet)
            if not ready:
                break
            os.read(self.fd, 65536)
        self.buf = b""


def ask(port, command, expect_prefix=None):
    port.write(command)
    reply = port.readline()
    print(f"> {command}\n< {reply}")
    if expect_prefix and not reply.startswith(expect_prefix):
        sys.exit(f"FAIL: {command!r} answered {reply!r}, expected {expect_prefix!r}")
    return reply


def capture(port, command):
    port.write(command)
    header = port.readline()
    if not header.startswith("DATA "):
        return header, None
    _, n, crc, us = header.split()
    n, crc, us = int(n), int(crc, 16), int(us)
    bits = 16 if command.startswith("CAP16") else 20 if command.startswith("CAP20") else 64
    size = n * 4 if bits == 64 else (n * bits + 7) // 8
    payload = port.read(size)
    if zlib.crc32(payload) != crc:
        sys.exit(f"FAIL: CRC mismatch for {command!r}")
    return header, payload


def iq8_stats(payload):
    i = [int.from_bytes(payload[k:k + 1], "little", signed=True) for k in range(0, len(payload), 2)]
    q = [-int.from_bytes(payload[k + 1:k + 2], "little", signed=True) for k in range(0, len(payload), 2)]
    n = len(i)
    mean_i, mean_q = sum(i) / n, sum(q) / n
    rms = math.sqrt(sum((a - mean_i) ** 2 + (b - mean_q) ** 2 for a, b in zip(i, q)) / n)
    distinct = len(set(zip(i, q)))
    return mean_i, mean_q, rms, distinct


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    port = Port(sys.argv[1])
    port.drain()
    nonce = str(int(time.time() * 1000))
    port.write(f"SYNC {nonce}")
    for _ in range(50):
        if port.readline(timeout=10) == f"SYNC {nonce}":
            break
    else:
        sys.exit("FAIL: no SYNC echo")
    print(f"synced with nonce {nonce}")

    ask(port, "INFO", "S3SDR 6 burst")
    ask(port, "CAPS", "CAPS ")
    ask(port, "VERSION?", "VERSION {")
    ask(port, "TRANSPORT?", "TRANSPORT ")
    ask(port, "RANGE?", "RANGE 100 6000 1")
    ask(port, "LIMITS?", "LIMITS {")
    ask(port, "GAIN?", "GAIN ")
    ask(port, "FREQ 2442", "OK")
    ask(port, "NOPE", "ERR command")
    ask(port, "FOFS 5000", "ERR args")

    for rate, name in [(0, "80"), (1, "40"), (6, "16")]:
        header, payload = capture(port, f"CAP16 4096 {rate}")
        if payload is None:
            sys.exit(f"FAIL: CAP16 at {name} MS/s answered {header!r}")
        mi, mq, rms, distinct = iq8_stats(payload)
        print(f"< {header}  [{name} MS/s] mean=({mi:.1f},{mq:.1f}) rms={rms:.2f} distinct={distinct}")
        if distinct < 8:
            sys.exit("FAIL: samples look stuck (fewer than 8 distinct I/Q values)")

    header, payload = capture(port, "CAP20 16380 0")
    if payload is None:
        sys.exit(f"FAIL: CAP20 answered {header!r}")
    print(f"< {header}  [IQ10, full length]")
    header, payload = capture(port, "CAP 1024 0")
    if payload is None:
        sys.exit(f"FAIL: CAP answered {header!r}")
    gains = {int.from_bytes(payload[k:k + 4], "little") >> 20 & 0xFF for k in range(0, len(payload), 4)}
    print(f"< {header}  [raw words, gain field values {sorted(gains)[:8]}]")

    ask(port, "CAP16 4096 3", "ERR rate")
    ask(port, "GAIN MANUAL 40", "OK")
    ask(port, "GAIN?", "GAIN MANUAL 40")
    header, payload = capture(port, "CAP 256 0")
    gains = {int.from_bytes(payload[k:k + 4], "little") >> 20 & 0xFF for k in range(0, len(payload), 4)}
    print(f"< {header}  [manual gain 40, gain field {sorted(gains)}]")
    ask(port, "GAIN HARDWARE", "OK")
    ask(port, "BANDWIDTH 20", "OK")
    ask(port, "LPF?", "LPF ")
    header, payload = capture(port, "CAP16 4096 0")
    print(f"< {header}  [BANDWIDTH 20]")
    ask(port, "LPF AUTO", "OK")
    ask(port, "FREQ 2000", "OK")
    header, payload = capture(port, "CAP16 4096 0")
    if payload is None:
        sys.exit(f"FAIL: capture at 2000 MHz answered {header!r}")
    print(f"< {header}  [2000 MHz through the 5/6 divider]")
    ask(port, "FREQ 2442", "OK")

    port.write("RXRUN 1024 0 3 16")
    for k in range(3):
        header = port.readline()
        _, n, crc, _ = header.split()
        if zlib.crc32(port.read(int(n) * 2)) != int(crc, 16):
            sys.exit("FAIL: RXRUN CRC mismatch")
    end = port.readline()
    print(f"> RXRUN 1024 0 3 16\n< 3 x DATA, then {end}")
    if end != "END":
        sys.exit("FAIL: RXRUN did not end with END")
    ask(port, "RELEASE", "OK")
    print("PASS")


if __name__ == "__main__":
    main()
