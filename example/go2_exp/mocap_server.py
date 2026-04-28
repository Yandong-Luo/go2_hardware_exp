#!/usr/bin/env python3
"""
Vicon MoCap -> ZMQ publisher.

Publishes robot and tracked-point poses over a ZMQ PUB socket on port 5555 in the
JSON schema consumed by go2_pid.cpp::updatePoseFromMocap. Run this on the machine
whose IP go2_pid.cpp connects to (currently 143.215.96.245).

Dependencies:
    pip install pyzmq numpy
    pip install vicon-dssdk          # official Vicon Python SDK

The SDK is normally distributed with Vicon Tracker / Nexus. On Linux the
`vicon_dssdk` wheel is shipped under <Tracker>/SDK/Python or can be installed
from the Vicon support site.
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import zmq

try:
    from vicon_dssdk import ViconDataStream
except ImportError:
    ViconDataStream = None


# Subjects we care about. Edit these names so they match the subject names
# configured in Vicon Tracker.
ROBOT_SUBJECTS = ["Go2"]                 # 6-DoF subjects -> "robots" field
POINT_SUBJECTS = {"Go2_head": "Go2_head"}  # subject_name -> key in "camera_positions"


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (rotation about Z) from a unit quaternion, in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ViconClient:
    """Thin wrapper that pulls a single frame of pose data from Vicon."""

    def __init__(self, host: str):
        if ViconDataStream is None:
            raise RuntimeError(
                "vicon_dssdk not installed. Install it from your Vicon "
                "Tracker/Nexus distribution, or run with --mock for testing."
            )

        self.client = ViconDataStream.Client()
        print(f"Connecting to Vicon DataStream at {host} ...")
        self.client.Connect(host)

        # Pull translations in metres (SDK default is millimetres).
        self.client.SetAxisMapping(
            ViconDataStream.Client.AxisMapping.EForward,
            ViconDataStream.Client.AxisMapping.ELeft,
            ViconDataStream.Client.AxisMapping.EUp,
        )
        self.client.EnableSegmentData()
        self.client.EnableMarkerData()
        self.client.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)

    def get_frame(self):
        """Block until a fresh frame is available, then return it as a dict."""
        while not self.client.GetFrame():
            time.sleep(0.001)

        robots = {}
        for name in ROBOT_SUBJECTS:
            try:
                trans, occluded = self.client.GetSegmentGlobalTranslation(name, name)
                rot, _ = self.client.GetSegmentGlobalRotationQuaternion(name, name)
            except ViconDataStream.DataStreamException:
                continue
            if occluded:
                continue

            # Vicon translations are in mm by default — convert to metres.
            x, y, z = (v / 1000.0 for v in trans)
            qx, qy, qz, qw = rot
            yaw = quat_to_yaw(qx, qy, qz, qw)

            robots[name] = {
                "position": [x, y, z],
                "rotation": [qx, qy, qz, qw],
                "yaw": yaw,
            }

        camera_positions = {}
        for subject, key in POINT_SUBJECTS.items():
            try:
                trans, occluded = self.client.GetSegmentGlobalTranslation(subject, subject)
            except ViconDataStream.DataStreamException:
                continue
            if occluded:
                continue
            camera_positions[key] = [v / 1000.0 for v in trans]

        return {
            "timestamp": time.time(),
            "robots": robots,
            "camera_positions": camera_positions,
        }


class MockClient:
    """Generates a synthetic circular trajectory so the server can be tested
    without a live Vicon system."""

    def __init__(self):
        self.t0 = time.time()

    def get_frame(self):
        t = time.time() - self.t0
        x, y, z = math.cos(0.5 * t), math.sin(0.5 * t), 0.3
        yaw = 0.5 * t
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        return {
            "timestamp": time.time(),
            "robots": {
                "Go2": {
                    "position": [x, y, z],
                    "rotation": [0.0, 0.0, qz, qw],
                    "yaw": yaw,
                }
            },
            "camera_positions": {"Go2_head": [x, y, z + 0.25]},
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vicon-host",
        default="localhost:801",
        help="Vicon DataStream host:port (default: localhost:801)",
    )
    parser.add_argument(
        "--bind",
        default="tcp://*:5555",
        help="ZMQ bind address (default: tcp://*:5555)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="Maximum publish rate in Hz (default: 100)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run with a synthetic trajectory instead of connecting to Vicon.",
    )
    args = parser.parse_args()

    client = MockClient() if args.mock else ViconClient(args.vicon_host)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(args.bind)
    print(f"Publishing MoCap frames on {args.bind}")

    period = 1.0 / args.rate
    last_log = time.time()
    frames_sent = 0

    try:
        while True:
            loop_start = time.time()
            frame = client.get_frame()
            sock.send_string(json.dumps(frame))
            frames_sent += 1

            now = time.time()
            if now - last_log >= 2.0:
                hz = frames_sent / (now - last_log)
                bots = list(frame["robots"].keys())
                print(f"[{now:.1f}] {hz:.1f} Hz, robots tracked: {bots}")
                frames_sent = 0
                last_log = now

            sleep_for = period - (time.time() - loop_start)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    sys.exit(main())
