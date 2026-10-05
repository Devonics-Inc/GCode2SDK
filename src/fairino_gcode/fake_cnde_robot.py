#!/usr/bin/env python3
"""
fake_cnde_robot.py - a pretend FAIRINO controller for trying the GUI without the arm.

It answers the CNDE status protocol on UDP the way fairino_CNDE_listener.py expects (configure,
start, stop, version) and streams a TCP position that moves on its own:

    python3 fake_cnde_robot.py                                  # TCP runs round a circle
    python3 fake_cnde_robot.py --gcode samples/demo_part.gcode  # TCP traces that file's plan
    python3 gcode_gui.py samples/demo_part.gcode                # then Connect... to 127.0.0.1

This is a test tool. It is written from the client script, not from a real controller, so it proves
the GUI's plumbing and nothing about how a real robot behaves.
"""
import argparse
import math
import socket
import struct
import time

from . import fairino_CNDE_listener as cnde

# item name -> data type, as in the SDK's CNDE state table
TYPES = {
    "actual_TCP_pos": "DOUBLE_6", "actual_joint_pos": "DOUBLE_6", "actual_joint_vel": "DOUBLE_6",
    "actual_TCP_cmpvel": "DOUBLE_2", "tool_id": "INT32", "wobj_id": "INT32",
    "robot_mode": "UINT8", "robot_state": "UINT8", "program_state": "UINT8", "motion_done": "UINT8",
    "main_code": "INT32", "sub_code": "INT32", "emergency_stop": "UINT8",
    "std_DI_box": "UINT8", "std_DO_box": "UINT8", "timestamp_us": "UINT64",
}
LEGACY = {"actual_TCP_pos", "actual_joint_pos", "actual_joint_vel", "robot_mode", "program_state",
          "motion_done", "main_code", "sub_code", "emergency_stop", "std_DI_box", "std_DO_box",
          "timestamp_us"}


def circle_path(radius=60.0, centre=(300.0, 0.0, 200.0), speed=40.0):
    """(seconds) -> (xyz, speed mm/s, moving) on a horizontal circle."""
    def at(t):
        a = speed * t / radius
        return (centre[0] + radius * math.cos(a), centre[1] + radius * math.sin(a), centre[2]), speed, True
    return at


def gcode_path(path):
    """(seconds) -> (xyz, speed, moving): the file's plan at its own speeds, 2 s pause, repeat."""
    from . import gcode_to_fairino as eng
    from .plan_model import build_plan, default_settings
    plan = build_plan(path, default_settings())
    legs, t = [], 0.0                     # (t0, t1, p0, p1, speed)
    last = None
    for k in plan.motions:
        speed = eng.speed_mm_s(plan.cmds[k], plan.settings)
        for p in plan.paths[k]:
            p = tuple(float(v) for v in p)
            if last is not None and math.dist(last, p) > 1e-9:
                dt = math.dist(last, p) / speed
                legs.append((t, t + dt, last, p, speed))
                t += dt
            last = p
    total = t + 2.0

    def at(now):
        s = now % total
        for t0, t1, p0, p1, speed in legs:
            if s < t1:
                f = (s - t0) / (t1 - t0)
                return tuple(a + (b - a) * f for a, b in zip(p0, p1)), speed, True
        return legs[-1][3], 0.0, False
    return at


def serve(port, path_at, legacy=False, fault_after=None, verbose=True, stop=None):
    known = LEGACY if legacy else set(TYPES)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    names, period, client, streaming, count = [], 0.1, None, False, 0
    t_start = next_send = time.monotonic()

    def reply(addr, msg_type, text=""):
        nonlocal count
        count = (count + 1) & 0xFF
        sock.sendto(cnde.build_frame(cnde.FT_MESSAGE, bytes([msg_type]) + text.encode(), count), addr)

    if verbose:
        print(f"fake CNDE robot listening on UDP {port}{' (legacy item names only)' if legacy else ''}")
    while stop is None or not stop.is_set():
        sock.settimeout(max(0.001, next_send - time.monotonic()) if streaming else 0.2)
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            data = None
        if data:
            for _n, ftype, content in cnde.iter_frames(data):
                if ftype == cnde.FT_OUTPUT_CFG:
                    asked = content[2:].decode("ascii", "replace").split(",")
                    unknown = [n for n in asked if n not in known]
                    if unknown:
                        reply(addr, cnde.MSG_ERROR, "unknown output name: " + ",".join(unknown))
                        continue
                    period = struct.unpack_from("<H", content)[0] / 1000.0
                    names, client = asked, addr
                    reply(addr, cnde.MSG_SUCCESS, ",".join(TYPES[n] for n in names))
                    if verbose:
                        print(f"configured by {addr[0]}: {len(names)} items every {period * 1000:.0f} ms")
                elif ftype == cnde.FT_OUTPUT_START:
                    streaming, next_send = bool(names), time.monotonic()
                    reply(addr, cnde.MSG_SUCCESS)
                elif ftype == cnde.FT_OUTPUT_STOP:
                    streaming = False
                    reply(addr, cnde.MSG_SUCCESS)
                elif ftype == cnde.FT_GET_VERSION:
                    reply(addr, cnde.MSG_SUCCESS, "fake-cnde-robot 1.0")
        now = time.monotonic()
        if streaming and now >= next_send:
            next_send = max(next_send + period, now)
            t = now - t_start
            xyz, speed, moving, *extra = path_at(t)      # xyz may be a full pose; extra: item overrides
            pose = [float(v) for v in xyz] + [180.0, 0.0, 0.0][len(xyz) - 3:]
            fault = fault_after is not None and t > fault_after
            values = {
                "actual_TCP_pos": pose, "actual_joint_pos": [0.0] * 6,
                "actual_joint_vel": [0.0] * 6, "actual_TCP_cmpvel": [speed, 0.0], "tool_id": 1, "wobj_id": 0,
                "robot_mode": 0, "robot_state": 2 if moving else 1, "program_state": 2 if moving else 1,
                "motion_done": 0 if moving else 1, "main_code": 14 if fault else 0, "sub_code": 3 if fault else 0,
                "emergency_stop": 0, "std_DI_box": 0, "std_DO_box": 0, "timestamp_us": int(now * 1e6),
            }
            values.update(extra[0] if extra else {})
            payload = b""
            for n in names:
                base, cnt = cnde.split_type(TYPES[n])
                v = values[n]
                payload += struct.pack("<" + cnde.STRUCT_CODE[base] * cnt, *(v if cnt > 1 else [v]))
            count = (count + 1) & 0xFF
            sock.sendto(cnde.build_frame(cnde.FT_OUTPUT_DATA, payload, count), client)
    sock.close()


def main():
    ap = argparse.ArgumentParser(description="Pretend FAIRINO controller (CNDE status stream, UDP).")
    ap.add_argument("--port", type=int, default=20006)
    ap.add_argument("--gcode", help="trace this G-code file's plan instead of a circle")
    ap.add_argument("--legacy", action="store_true", help="only know the older item names")
    ap.add_argument("--fault-after", type=float, help="report a fault code after this many seconds")
    args = ap.parse_args()
    try:
        serve(args.port, gcode_path(args.gcode) if args.gcode else circle_path(), args.legacy, args.fault_after)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
