#!/usr/bin/env python3
"""
CNDE listener for FAIRINO collaborative robots.

Implements the client side of the Configurable Network Data Exchange protocol:

  1. send an Output Configuration frame  (frame type 0x01)
  2. read back the Message frame         (frame type 0x06) containing the
     comma-separated data types of everything that was configured
  3. send CNDE output start              (frame type 0x02)
  4. receive Output Data frames          (frame type 0x04) and unpack them
     according to the types from step 2
  5. send CNDE output stop               (frame type 0x03) on exit

Frame layout (Table 2-1), all multi-byte fields little-endian:

    0x5A5A | count(1) | type(1) | len(2) | content(len) | 0xA5A5

Usage
-----
    python3 cnde_listener.py -- 192.168.58.2
    python3 cnde_listener.py --ip 192.168.58.2 --period 20 \
        --items actual_joint_pos,actual_TCP_pos,robot_state,motion_done
    python3 cnde_listener.py --ip 192.168.58.2 --raw          # hex dump only
    python3 cnde_listener.py --ip 192.168.58.2 --version      # firmware info

Tested against the protocol description in the FAIRINO "Robot Communication"
manual, sections 1-3.  UDP port 20006 (20005 is the TCP variant of the same
protocol; pass --tcp to use it).
"""

import argparse
import socket
import struct
import sys
import time

# --------------------------------------------------------------------------
# Protocol constants
# --------------------------------------------------------------------------

HEADER = b"\x5a\x5a"
TAIL = b"\xa5\xa5"
HEADER_LEN = 6            # header(2) + count(1) + type(1) + length(2)
TAIL_LEN = 2

# Table 2-2 - frame types
FT_INPUT_CFG = 0x00       # client -> robot
FT_OUTPUT_CFG = 0x01      # client -> robot
FT_OUTPUT_START = 0x02    # client -> robot
FT_OUTPUT_STOP = 0x03     # client -> robot
FT_OUTPUT_DATA = 0x04     # robot  -> client
FT_INPUT_DATA = 0x05      # client -> robot
FT_MESSAGE = 0x06         # both directions
FT_SET_VERSION = 0x07     # client -> robot
FT_GET_VERSION = 0x08     # both directions

FRAME_TYPE_NAMES = {
    FT_INPUT_CFG: "INPUT_CFG",
    FT_OUTPUT_CFG: "OUTPUT_CFG",
    FT_OUTPUT_START: "OUTPUT_START",
    FT_OUTPUT_STOP: "OUTPUT_STOP",
    FT_OUTPUT_DATA: "OUTPUT_DATA",
    FT_INPUT_DATA: "INPUT_DATA",
    FT_MESSAGE: "MESSAGE",
    FT_SET_VERSION: "SET_VERSION",
    FT_GET_VERSION: "GET_VERSION",
}

# Table 3-7 - message types
MSG_SUCCESS = 0x00
MSG_INFORMATION = 0x01
MSG_WARNING = 0x02
MSG_ERROR = 0x03
MSG_FAULT = 0x04

MSG_TYPE_NAMES = {
    MSG_SUCCESS: "SUCCESS",
    MSG_INFORMATION: "INFORMATION",
    MSG_WARNING: "WARNING",
    MSG_ERROR: "ERROR",
    MSG_FAULT: "FAULT",
}

# Table 1-3 - data type -> struct code (little-endian, no padding)
STRUCT_CODE = {
    "UINT8": "B",
    "INT8": "b",
    "UINT16": "H",
    "INT16": "h",
    "UINT32": "I",
    "INT32": "i",
    "UINT64": "Q",
    "INT64": "q",
    "FLOAT": "f",
    "DOUBLE": "d",
}

MAX_OUTPUT_BYTES = 4096   # robot side limit on the output data frame

# A reasonable default: joint + TCP feedback, program state, IO, fault codes.
DEFAULT_ITEMS = (
    "actual_joint_pos,"
    "actual_joint_vel,"
    "actual_TCP_pos,"
    "robot_mode,"
    "program_state,"
    "motion_done,"
    "std_DI_box,"
    "std_DO_box,"
    "main_code,"
    "sub_code,"
    "emergency_stop,"
    "timestamp_us"
)


# --------------------------------------------------------------------------
# Frame building / parsing
# --------------------------------------------------------------------------

def build_frame(frame_type: int, content: bytes, count: int) -> bytes:
    """Wrap `content` in a CNDE frame."""
    if len(content) > 0xFFFF:
        raise ValueError("content too long for a 16-bit length field")
    return (
        HEADER
        + struct.pack("<BBH", count & 0xFF, frame_type, len(content))
        + content
        + TAIL
    )


def iter_frames(buf: bytes):
    """Yield (count, frame_type, content) for every valid frame in `buf`.

    Scans for the 0x5A5A header instead of assuming the datagram starts on a
    frame boundary, so a datagram carrying several frames (or leading junk)
    still parses.
    """
    i = 0
    n = len(buf)
    while i + HEADER_LEN + TAIL_LEN <= n:
        start = buf.find(HEADER, i)
        if start < 0 or start + HEADER_LEN + TAIL_LEN > n:
            return
        count, ftype, length = struct.unpack_from("<BBH", buf, start + 2)
        end = start + HEADER_LEN + length
        if end + TAIL_LEN > n:
            return                      # truncated frame
        if buf[end:end + TAIL_LEN] != TAIL:
            i = start + 2               # bad tail, resync past this header
            continue
        yield count, ftype, buf[start + HEADER_LEN:end]
        i = end + TAIL_LEN


# --------------------------------------------------------------------------
# Data-type handling
# --------------------------------------------------------------------------

def split_type(token: str):
    """'DOUBLE_6' -> ('DOUBLE', 6);  'INT32' -> ('INT32', 1)."""
    token = token.strip()
    base, count = token, 1
    if "_" in token:
        head, _, tail = token.rpartition("_")
        if tail.isdigit():
            base, count = head, int(tail)
    if base not in STRUCT_CODE:
        raise ValueError("unknown data type %r reported by robot" % token)
    return base, count


class Layout:
    """Maps the robot's reply ('DOUBLE_6,UINT8,INT32') onto the byte stream."""

    def __init__(self, names, type_tokens):
        if len(names) != len(type_tokens):
            raise ValueError(
                "robot returned %d types for %d configured names"
                % (len(type_tokens), len(names))
            )
        self.names = names
        self.fields = []              # (name, base_type, count)
        fmt = "<"
        for name, token in zip(names, type_tokens):
            base, cnt = split_type(token)
            self.fields.append((name, base, cnt))
            fmt += STRUCT_CODE[base] * cnt
        self.struct = struct.Struct(fmt)
        self.size = self.struct.size

    def unpack(self, payload: bytes) -> dict:
        if len(payload) < self.size:
            raise ValueError(
                "short output frame: got %d bytes, expected %d"
                % (len(payload), self.size)
            )
        flat = self.struct.unpack_from(payload, 0)
        out, pos = {}, 0
        for name, base, cnt in self.fields:
            chunk = flat[pos:pos + cnt]
            pos += cnt
            out[name] = chunk[0] if cnt == 1 else list(chunk)
        return out

    def describe(self) -> str:
        lines, off = [], 0
        for name, base, cnt in self.fields:
            width = struct.calcsize("<" + STRUCT_CODE[base]) * cnt
            type_str = base if cnt == 1 else "%s_%d" % (base, cnt)
            lines.append("  %4d  %-32s %-12s %d byte(s)"
                         % (off, name, type_str, width))
            off += width
        lines.append("  total: %d bytes" % self.size)
        return "\n".join(lines)


# --------------------------------------------------------------------------
# Client
# --------------------------------------------------------------------------

class CNDEClient:
    def __init__(self, ip, port=20006, use_tcp=False, timeout=2.0):
        self.ip = ip
        self.port = port
        self.use_tcp = use_tcp
        self.timeout = timeout
        self._count = 0
        if use_tcp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(timeout)
            self.sock.connect((ip, port))
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(timeout)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)

    # -- low level -------------------------------------------------------
    def _next_count(self):
        self._count = (self._count + 1) & 0xFF
        return self._count

    def send(self, frame_type, content=b""):
        frame = build_frame(frame_type, content, self._next_count())
        if self.use_tcp:
            self.sock.sendall(frame)
        else:
            self.sock.sendto(frame, (self.ip, self.port))

    def recv_frame(self, want_types=None, timeout=None):
        """Block until a frame of one of `want_types` arrives. Returns
        (count, type, content) or None on timeout."""
        deadline = time.time() + (self.timeout if timeout is None else timeout)
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                if self.use_tcp:
                    data = self.sock.recv(65535)
                    if not data:
                        raise ConnectionError("robot closed the TCP connection")
                else:
                    data, addr = self.sock.recvfrom(65535)
                    if addr[0] != self.ip:
                        continue
            except socket.timeout:
                return None
            for count, ftype, content in iter_frames(data):
                if want_types is None or ftype in want_types:
                    return count, ftype, content

    def drain(self):
        """Throw away anything already queued (e.g. a stream left running)."""
        self.sock.settimeout(0.0)
        try:
            while True:
                if self.use_tcp:
                    if not self.sock.recv(65535):
                        break
                else:
                    self.sock.recvfrom(65535)
        except (socket.timeout, BlockingIOError, OSError):
            pass
        finally:
            self.sock.settimeout(self.timeout)

    # -- protocol steps ---------------------------------------------------
    @staticmethod
    def _decode_message(content: bytes):
        if not content:
            return None, ""
        return content[0], content[1:].decode("utf-8", "replace").strip("\x00")

    def configure_output(self, names, period_ms):
        """Send the output configuration frame (Table 3-3) and build a Layout
        from the robot's reply."""
        if not 1 <= period_ms <= 200:
            raise ValueError("output period must be 1-200 ms")
        name_str = ",".join(names)
        content = struct.pack("<H", period_ms) + name_str.encode("ascii")
        self.send(FT_OUTPUT_CFG, content)

        reply = self.recv_frame({FT_MESSAGE})
        if reply is None:
            raise TimeoutError(
                "no reply to the output configuration frame - check the IP, "
                "that port %d is reachable, and that the robot is powered on"
                % self.port
            )
        msg_type, text = self._decode_message(reply[2])
        if msg_type != MSG_SUCCESS:
            raise RuntimeError(
                "robot rejected the output configuration [%s]: %s"
                % (MSG_TYPE_NAMES.get(msg_type, hex(msg_type)), text)
            )
        layout = Layout(names, [t for t in text.split(",") if t.strip()])
        if layout.size > MAX_OUTPUT_BYTES:
            raise RuntimeError(
                "configured output is %d bytes, over the %d byte limit"
                % (layout.size, MAX_OUTPUT_BYTES)
            )
        return layout

    def start_output(self):
        self.send(FT_OUTPUT_START)
        reply = self.recv_frame({FT_MESSAGE})
        if reply is not None:
            msg_type, text = self._decode_message(reply[2])
            if msg_type != MSG_SUCCESS:
                raise RuntimeError("start rejected [%s]: %s"
                                   % (MSG_TYPE_NAMES.get(msg_type), text))

    def stop_output(self):
        try:
            self.send(FT_OUTPUT_STOP)
            self.recv_frame({FT_MESSAGE}, timeout=0.5)
        except OSError:
            pass

    def firmware_version(self):
        self.send(FT_GET_VERSION)
        reply = self.recv_frame({FT_GET_VERSION, FT_MESSAGE})
        if reply is None:
            return None
        content = reply[2]
        # the reply comes back as a message: type byte + string
        if reply[1] == FT_MESSAGE:
            return self._decode_message(content)[1]
        return content.decode("utf-8", "replace").strip("\x00")

    def stream(self, layout=None):
        """Yield decoded samples (or raw bytes when layout is None) forever."""
        while True:
            frame = self.recv_frame({FT_OUTPUT_DATA, FT_MESSAGE}, timeout=5.0)
            if frame is None:
                print("[warn] no output data for 5 s", file=sys.stderr)
                continue
            count, ftype, content = frame
            if ftype == FT_MESSAGE:
                mtype, text = self._decode_message(content)
                print("[robot %s] %s" % (MSG_TYPE_NAMES.get(mtype, mtype), text),
                      file=sys.stderr)
                continue
            yield count, (content if layout is None else layout.unpack(content))

    def close(self):
        self.sock.close()


# --------------------------------------------------------------------------
# Formatting
# --------------------------------------------------------------------------

def format_value(name, value):
    if isinstance(value, list):
        if all(isinstance(v, float) for v in value):
            return "[" + ", ".join("%9.3f" % v for v in value) + "]"
        return "[" + ", ".join(str(v) for v in value) + "]"
    if isinstance(value, float):
        return "%.3f" % value
    # show the byte-packed IO words as bits too
    if name.endswith(("_DI_box", "_DO_box", "_DI_tool", "_DO_tool")):
        return "%d (0b%08d)" % (value, int(bin(value)[2:]))
    return str(value)


def print_sample(count, sample, sep=True):
    if sep:
        print("-" * 60)
    print("frame #%03d  %s" % (count, time.strftime("%H:%M:%S")))
    width = max(len(k) for k in sample)
    for name, value in sample.items():
        print("  %-*s : %s" % (width, name, format_value(name, value)))


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Listen to a FAIRINO robot's CNDE status stream.")
    ap.add_argument("--ip", default="192.168.58.25", help="robot IP address")
    ap.add_argument("--port", type=int, default=None,
                    help="default 20006 for UDP, 20005 for TCP")
    ap.add_argument("--tcp", action="store_true",
                    help="use the TCP port instead of UDP")
    ap.add_argument("--period", type=int, default=100,
                    help="output period in ms (1-200), default 100")
    ap.add_argument("--items", default=DEFAULT_ITEMS,
                    help="comma-separated output function names (Table 1-1)")
    ap.add_argument("--count", type=int, default=0,
                    help="stop after N samples (0 = run until Ctrl-C)")
    ap.add_argument("--raw", action="store_true",
                    help="hex-dump the payload instead of decoding it")
    ap.add_argument("--csv", action="store_true",
                    help="write samples as CSV to stdout")
    ap.add_argument("--version", action="store_true",
                    help="just query the software/firmware version and exit")
    args = ap.parse_args()

    port = args.port if args.port else (20005 if args.tcp else 20006)
    names = [n.strip() for n in args.items.split(",") if n.strip()]

    client = CNDEClient(args.ip, port, use_tcp=args.tcp)
    print("connected to %s:%d (%s)" % (args.ip, port, "TCP" if args.tcp else "UDP"),
          file=sys.stderr)

    try:
        if args.version:
            print(client.firmware_version() or "<no reply>")
            return 0

        client.drain()
        client.stop_output()      # in case a previous run left it streaming
        client.drain()

        layout = client.configure_output(names, args.period)
        print("configuration accepted:\n%s" % layout.describe(), file=sys.stderr)

        client.start_output()
        print("streaming every %d ms - Ctrl-C to stop\n" % args.period,
              file=sys.stderr)

        header_written = False
        n = 0
        for count, sample in client.stream(None if args.raw else layout):
            if args.raw:
                print("frame #%03d  %d bytes\n%s" % (count, len(sample), sample.hex(" ")))
            elif args.csv:
                if not header_written:
                    cols = []
                    for fname, _, cnt in layout.fields:
                        cols += ([fname] if cnt == 1
                                 else ["%s[%d]" % (fname, i) for i in range(cnt)])
                    print("time," + ",".join(cols))
                    header_written = True
                row = []
                for value in sample.values():
                    row += value if isinstance(value, list) else [value]
                print("%.3f," % time.time() + ",".join(str(v) for v in row))
            else:
                print_sample(count, sample)
            n += 1
            if args.count and n >= args.count:
                break
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    except (RuntimeError, TimeoutError, ValueError, ConnectionError) as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1
    finally:
        client.stop_output()
        client.close()
        print("output stopped", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
