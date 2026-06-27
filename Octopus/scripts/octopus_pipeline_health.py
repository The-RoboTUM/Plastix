#!/usr/bin/env python3

import json
import subprocess
import sys
import time
import urllib.request
from typing import Optional


def run(cmd, timeout=4):
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except subprocess.TimeoutExpired:
        return 124, "", "timeout"


def ok(msg):
    print(f"✅ {msg}")


def warn(msg):
    print(f"⚠️  {msg}")


def bad(msg):
    print(f"❌ {msg}")


def info(msg):
    print(f"   {msg}")


def topic_info(topic: str) -> bool:
    code, out, err = run(["ros2", "topic", "info", topic, "-v"], timeout=4)
    if code != 0 or "Type:" not in out:
        bad(f"{topic}: missing")
        if err:
            info(err)
        return False

    publisher_count = "Publisher count: 0" not in out
    subscription_count = "Subscription count: 0" not in out

    if publisher_count:
        ok(f"{topic}: publisher exists")
    else:
        warn(f"{topic}: no publisher")

    if subscription_count:
        ok(f"{topic}: subscriber exists")
    else:
        warn(f"{topic}: no subscriber")

    for line in out.splitlines():
        if "Type:" in line or "Publisher count:" in line or "Subscription count:" in line or "Node name:" in line:
            info(line)

    return publisher_count


def topic_once(topic: str, msg_type: Optional[str] = None, field: Optional[str] = None, timeout=5):
    cmd = ["ros2", "topic", "echo", "--once", topic]
    if msg_type:
        cmd.append(msg_type)
    if field:
        cmd += ["--field", field]

    code, out, err = run(cmd, timeout=timeout)
    if code == 0 and out:
        ok(f"{topic}: received one message")
        preview = out[:500].replace("\n", " ")
        info(preview + ("..." if len(out) > 500 else ""))
        return True

    warn(f"{topic}: no message within {timeout}s")
    if err:
        info(err[:300])
    return False


def http_json(url: str):
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
            body = r.read().decode("utf-8", errors="replace")
        data = json.loads(body)
        ok(f"{url}: reachable")
        preview = json.dumps(data, indent=2)[:700]
        info(preview + ("..." if len(preview) >= 700 else ""))
        return data
    except Exception as e:
        bad(f"{url}: not reachable ({e})")
        return None


def main():
    print("\n=== Octopus Pipeline Health Check ===\n")

    print("ROS environment:")
    code, out, _ = run(["bash", "-lc", "echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"], timeout=2)
    info(out)
    print()

    print("Core input topics:")
    topic_info("/camera/image_raw/compressed")
    print()
    topic_info("/fmu/out/vehicle_odometry")
    print()
    topic_info("/fmu/out/vehicle_local_position")
    print()

    print("Detector topics:")
    topic_info("/detector_node/confirmed")
    print()
    topic_info("/detector_node/debug_image/compressed")
    print()
    topic_info("/detector_node/detections_debug")
    print()

    print("Transform / map topics:")
    topic_info("/octopus/flight_camera_transform/status")
    topic_once("/octopus/flight_camera_transform/status", "std_msgs/msg/String", "data", timeout=4)
    print()
    topic_info("/octopus/detections_world_pose")
    print()
    topic_info("/octopus/detections_world")
    print()
    topic_info("/octopus/map_patch")
    print()

    print("Backend endpoints:")
    http_json("http://127.0.0.1:8000/api/camera_debug/latest")
    print()
    http_json("http://127.0.0.1:8000/api/map_patch/latest")

    print("\n=== Done ===")


if __name__ == "__main__":
    main()
