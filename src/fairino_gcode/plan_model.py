#!/usr/bin/env python3
"""
plan_model.py - a G-code file turned into the robot instructions that would be sent, plus what a
viewer needs to draw it. No GUI code in here; it only uses the engine (gcode_to_fairino.py).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import gcode_to_fairino as eng


@dataclass
class Plan:
    path: str
    settings: argparse.Namespace
    cmds: List[eng.Cmd]
    warnings: List[str]
    frame: eng.Frame
    follow: bool                                   # posture follows the path tangent
    summary: List[str]
    bbox: Optional[Tuple[list, list]]
    paths: Dict[int, np.ndarray] = field(default_factory=dict)   # command index -> (N, 3) TCP points
    placed: bool = False                           # True: positioned with the robot (real coordinates)
    unreachable: List[list] = field(default_factory=list)        # XYZ of targets the robot cannot reach

    @property
    def motions(self) -> List[int]:
        return sorted(self.paths)


def default_settings() -> argparse.Namespace:
    """The command-line tool's defaults, except that G-code XYZ are shown as they are (origin 0,0,0,
    no approach move) - the coordinates the robot sees in WebApp mode, in the workpiece frame."""
    a = eng.build_parser().parse_args(["-"])
    a.origin = [0.0, 0.0, 0.0]
    a.approach = 0.0
    return a


def build_plan(path: str, settings: argparse.Namespace, frame: Optional[eng.Frame] = None) -> Plan:
    """Parse and plan a file. Raises OSError (cannot read) or ValueError (cannot convert).

    frame=None places the drawing by settings.origin / rpy / yaw / scale (no robot involved).
    Pass the frame from eng.resolve_frame() to get the plan where the robot will really run it.
    """
    with open(path, "r", errors="replace") as f:
        cmds, warnings = eng.parse_gcode(f.read(), settings.feed, settings.dwell_ms, settings.arc_split)
    placed = frame is not None
    if frame is None:
        frame = eng.Frame(settings.origin, settings.rpy, settings.yaw, settings.scale)
    follow = eng.plan(cmds, frame, settings)
    summary: List[str] = []
    bbox = eng.summarize(cmds, frame, settings, follow, out=summary.append)
    plan = Plan(path, settings, cmds, warnings, frame, follow, summary, bbox, placed=placed)
    first = True
    for k, c in enumerate(cmds):
        if c.kind not in eng.MOTION_KINDS:
            continue
        pts = np.array([frame.pose(p)[:3] for p in eng.polyline(c)], dtype=float)
        if first and c.rapid:
            pts = pts[-1:]            # the arm comes from wherever it is, not from G-code 0,0,0
        first = False
        plan.paths[k] = pts
    return plan
