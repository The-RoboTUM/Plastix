#!/usr/bin/env python3

"""Verify the rosbridge link the GripperX team asked for, from the client side.

This is step 5 of ``OCTOPUS_ROSBRIDGE_SETUP.md`` turned into one runnable check.
It needs no ROS - only Python and the ``websockets`` package - so it can be run
from the robot's laptop to test the firewall rule as well as the link:

    python3 check_rosbridge.py                 # against 127.0.0.1
    python3 check_rosbridge.py 10.0.0.42       # against the Octopus host

It speaks exactly what the GripperX client says it speaks: ``subscribe``,
``advertise`` and ``publish``, with the short type form (``std_msgs/String``,
not ``std_msgs/msg/String``). No rosapi, no services, no parameters.

Both directions are exercised:

  Octopus -> robot   the three outbound topics must deliver ``publish`` frames
  robot -> Octopus   ``/octopus/trash_goal_done`` and the nested telemetry topic
                     ``/octopus/devices/gripperx/status`` must be publishable

The nested topic is the open question from the setup document: it answers
empirically whether the single glob entry ``/octopus/*`` spans ``/``.

The value published on ``trash_goal_done`` is deliberately not a well-formed
target id, so this check can never mark real litter as collected.
"""

import asyncio
import json
import sys
from collections import Counter

from websockets.asyncio.client import connect

PORT = 9090
COLLECT_SECONDS = 12.0

# Octopus -> robot. Short type form, exactly as the GripperX client sends it.
SUBSCRIBE_TOPICS = {
    "/octopus/fake_eve_gps_start": "sensor_msgs/NavSatFix",
    "/octopus/trash_goal": "sensor_msgs/NavSatFix",
    "/octopus/trash_gps": "std_msgs/String",
}

# robot -> Octopus. The second entry is the nested-glob probe.
GOAL_DONE_TOPIC = "/octopus/trash_goal_done"
DEVICE_STATUS_TOPIC = "/octopus/devices/gripperx/status"

# Not a well-formed id, so trash_gps_goal_node ignores it with a warning
# instead of marking a real target collected.
SENTINEL = "connectivity-test-not-an-id"

DEVICE_STATUS_PROBE = {
    "source_id": "check_rosbridge.py",
    "robot_id": "gripperx",
    "pose": {"status": "no_datum", "frame_id": "map", "x": None, "y": None,
             "yaw_deg": None, "lat": None, "lon": None},
    "nav": {"status": "idle", "active_goal_id": None, "distance_remaining_m": None},
    "armed": False,
    "battery": {"status": "unavailable", "reason": "CONNECTIVITY_PROBE",
                "percent": None, "voltage_v": None},
    "link": {"connected": True, "last_rx_age_sec": 0.0},
    "note": "connectivity probe from check_rosbridge.py, not a real robot",
}


async def run(host):
    url = f"ws://{host}:{PORT}"
    print(f"connecting to {url}")

    publish_counts = Counter()
    status_frames = []

    async with connect(url, max_size=2 ** 20) as ws:
        print("connected\n")

        for topic, msg_type in SUBSCRIBE_TOPICS.items():
            await ws.send(json.dumps({"op": "subscribe", "id": f"sub:{topic}",
                                      "topic": topic, "type": msg_type}))

        for topic, msg_type in ((GOAL_DONE_TOPIC, "std_msgs/String"),
                                (DEVICE_STATUS_TOPIC, "std_msgs/String")):
            await ws.send(json.dumps({"op": "advertise", "id": f"adv:{topic}",
                                      "topic": topic, "type": msg_type}))

        # Let the advertises register before publishing onto them.
        await asyncio.sleep(1.0)

        await ws.send(json.dumps({"op": "publish", "id": "pub:goal_done",
                                  "topic": GOAL_DONE_TOPIC,
                                  "msg": {"data": SENTINEL}}))
        await ws.send(json.dumps({"op": "publish", "id": "pub:device_status",
                                  "topic": DEVICE_STATUS_TOPIC,
                                  "msg": {"data": json.dumps(DEVICE_STATUS_PROBE)}}))

        print(f"collecting for {COLLECT_SECONDS:.0f}s ...")
        loop = asyncio.get_event_loop()
        deadline = loop.time() + COLLECT_SECONDS
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                break
            try:
                frame = json.loads(await asyncio.wait_for(ws.recv(), timeout=remaining))
            except asyncio.TimeoutError:
                break
            op = frame.get("op")
            if op == "publish":
                publish_counts[frame.get("topic")] += 1
            elif op == "status":
                status_frames.append(frame)
                print(f"  status [{frame.get('level')}] {frame.get('msg')}")

    print()
    print("Octopus -> robot")
    ok = True
    for topic in SUBSCRIBE_TOPICS:
        count = publish_counts[topic]
        rate = count / COLLECT_SECONDS
        verdict = "OK" if count else "NO DATA"
        if not count:
            ok = False
        print(f"  {verdict:<8} {topic:<34} {count:>3} frames (~{rate:.1f} Hz)")

    errors = [f for f in status_frames if f.get("level") == "error"]
    print()
    print("robot -> Octopus")
    for topic in (GOAL_DONE_TOPIC, DEVICE_STATUS_TOPIC):
        blamed = [f for f in errors if topic in str(f.get("msg", ""))]
        if blamed:
            ok = False
            print(f"  REJECTED {topic}")
            for frame in blamed:
                print(f"           {frame.get('msg')}")
        else:
            print(f"  ACCEPTED {topic}")
    print("  (confirm arrival on the graph with: ros2 topic echo <topic>)")

    if errors and not any(t in str(f.get("msg", "")) for f in errors
                          for t in (GOAL_DONE_TOPIC, DEVICE_STATUS_TOPIC)):
        ok = False
        print("\nunattributed error frames:")
        for frame in errors:
            print(f"  {frame.get('msg')}")

    print()
    print("RESULT:", "pass" if ok else "FAIL")
    return 0 if ok else 1


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    try:
        return asyncio.run(run(host))
    except OSError as exc:
        print(f"\nconnection to {host}:{PORT} failed: {exc}")
        print("rosbridge not running, or the port is not reachable (firewall).")
        return 1


if __name__ == "__main__":
    sys.exit(main())
