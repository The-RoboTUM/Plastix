#!/usr/bin/env python3

"""Stand in for the GripperX robot on the far side of the rosbridge link.

This is the counterpart of ``octopus-dashboard/SETUP_NO_DRONE.md``: it lets the
whole Octopus <-> GripperX loop be exercised with no robot and no ROS on the
client side, over exactly the transport the real robot will use.

    python3 simulate_gripperx.py                 # against 127.0.0.1
    python3 simulate_gripperx.py --host 10.0.0.5 # from another machine

What it does, in the same frames the real client speaks:

    subscribe  /octopus/fake_eve_gps_start   the datum it is started on
    subscribe  /octopus/trash_goal           where to drive next
    subscribe  /octopus/trash_gps            which target id that goal is
    publish    /octopus/devices/gripperx/status   its own state, 2 Hz
    publish    /octopus/trash_goal_done      when it "arrives"

It drives a straight line at a walking pace, so the marker visibly moves across
the mission map and the goal advances to the next piece of litter on arrival.

Until the datum arrives it reports ``pose.status = "no_datum"`` with null
lat/lon - the case the dashboard has to explain rather than draw.

    --no-collect   drive to the goal but never report it done. Use this when you
                   do not want the simulator to advance real detections.
"""

import argparse
import asyncio
import json
import math
import sys
import time

from websockets.asyncio.client import connect

STATUS_TOPIC = "/octopus/devices/gripperx/status"
GOAL_DONE_TOPIC = "/octopus/trash_goal_done"

# Same flat-earth approximation as the dashboard's localToLatLng() and
# trash_gps_goal_node, so a metre here is the same metre everywhere else.
METERS_PER_DEG_LAT = 111320.0

STATUS_PERIOD_SEC = 0.5
ARRIVAL_RADIUS_M = 0.25


def meters_per_deg_lon(lat):
    return METERS_PER_DEG_LAT * max(math.cos(math.radians(lat)), 1e-6)


def offset_meters(from_lat, from_lon, to_lat, to_lon):
    """Metres east/north from one coordinate to another."""
    east = (to_lon - from_lon) * meters_per_deg_lon(from_lat)
    north = (to_lat - from_lat) * METERS_PER_DEG_LAT
    return east, north


class Robot:
    """Just enough robot to be worth pointing a dashboard at."""

    def __init__(self, speed_mps, collect, dialect="dashboard"):
        self.speed_mps = speed_mps
        self.collect = collect
        self.dialect = dialect

        self.datum = None          # (lat, lon) - where it was switched on
        self.lat = None
        self.lon = None
        self.yaw_deg = 0.0

        self.goal = None           # (lat, lon)
        self.goal_id = None
        self.nav_status = "idle"
        self.distance_remaining = None
        self.reported_done = set()
        self.last_step = None

    # --- incoming ------------------------------------------------------

    def on_datum(self, msg):
        lat, lon = msg.get("latitude"), msg.get("longitude")
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            return
        first = self.datum is None
        self.datum = (lat, lon)
        if first:
            # A real robot is started at the datum, so that is where it begins.
            self.lat, self.lon = lat, lon
            print(f"datum {lat:.7f}, {lon:.7f} - starting there")

    def on_goal(self, msg):
        lat, lon = msg.get("latitude"), msg.get("longitude")
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            return
        if self.goal != (lat, lon):
            print(f"new goal {lat:.7f}, {lon:.7f}")
        self.goal = (lat, lon)

    def on_trash_gps(self, msg):
        try:
            payload = json.loads(msg.get("data") or "{}")
        except json.JSONDecodeError:
            return
        self.goal_id = payload.get("goal_id")

    # --- motion --------------------------------------------------------

    def step(self):
        """Advance towards the goal. Returns a target id to report, or None."""
        now = time.monotonic()
        dt = 0.0 if self.last_step is None else now - self.last_step
        self.last_step = now

        if self.lat is None or self.goal is None:
            self.nav_status = "idle" if self.lat is not None else "no_datum"
            self.distance_remaining = None
            return None

        east, north = offset_meters(self.lat, self.lon, self.goal[0], self.goal[1])
        distance = math.hypot(east, north)
        self.distance_remaining = distance

        if distance <= ARRIVAL_RADIUS_M:
            self.nav_status = "idle"
            if self.goal_id is not None and self.goal_id not in self.reported_done:
                self.reported_done.add(self.goal_id)
                if self.collect:
                    print(f"arrived - reporting target {self.goal_id} collected")
                    return self.goal_id
                print(f"arrived at target {self.goal_id} (--no-collect, not reporting)")
            return None

        self.nav_status = "navigating"
        self.yaw_deg = math.degrees(math.atan2(north, east))

        travel = min(self.speed_mps * dt, distance)
        if travel > 0:
            self.lat += (north / distance) * travel / METERS_PER_DEG_LAT
            self.lon += (east / distance) * travel / meters_per_deg_lon(self.lat)
        return None

    # --- outgoing ------------------------------------------------------

    def status_payload(self):
        has_fix = self.lat is not None
        pose = {
            "status": "ok" if has_fix else "no_datum",
            "frame_id": "map",
            "yaw_deg": round(self.yaw_deg, 1) if has_fix else None,
            "lat": self.lat,
            "lon": self.lon,
            "x": None,
            "y": None,
        }
        if has_fix and self.datum:
            east, north = offset_meters(self.datum[0], self.datum[1], self.lat, self.lon)
            pose["x"], pose["y"] = round(east, 3), round(north, 3)

        if self.dialect == "gripperx":
            return self._gripperx_dialect_payload(pose, has_fix)

        return {
            "source_id": "simulate_gripperx.py",
            "robot_id": "gripperx",
            "timestamp": time.time(),
            "pose": pose,
            "nav": {
                "status": self.nav_status,
                "active_goal_id": self.goal_id if self.nav_status == "navigating" else None,
                "distance_remaining_m": (
                    round(self.distance_remaining, 2)
                    if self.distance_remaining is not None else None
                ),
            },
            "armed": False,
            "battery": {
                "status": "unavailable",
                "reason": "NO_SENSOR_INSTALLED",
                "percent": None,
                "voltage_v": None,
            },
            "link": {"connected": True, "last_rx_age_sec": 0.0},
            "simulated": True,
        }

    def _gripperx_dialect_payload(self, pose, has_fix):
        """What the REAL robot puts on the wire (octopus_protocol.build_device_status).

        Different from the dashboard shape above in five places - `device_id` not
        `robot_id`, `stamp` not `timestamp`, flat `nav_state`/`active_goal_id`
        instead of a `nav` object, `link_ok` instead of `link.connected`, and two
        SEPARATE availability flags on the pose ("available"/"unavailable") where
        the dashboard has one ("ok"/"no_datum").

        device_status_backend_bridge_node translates it. Without this option that
        translation would first run against the real robot, which is a bad place
        to discover a field name.
        """
        return {
            "source_id": "simulate_gripperx.py",
            "device_id": "gripperx",
            "stamp": time.time(),
            "pose": {
                "status": "available",
                "reason": "",
                "lat": pose["lat"],
                "lon": pose["lon"],
                "latlon_status": "available" if has_fix else "unavailable",
                "latlon_reason": "" if has_fix else "NO_DATUM",
                "x": pose["x"],
                "y": pose["y"],
                "yaw_deg": pose["yaw_deg"],
                "speed_mps": 0.0,
            },
            "nav_state": self.nav_status,
            "nav_state_reason": "",
            "active_goal_id": self.goal_id if self.nav_status == "navigating" else None,
            "armed": False,
            "arming_seconds_remaining": None,
            "last_disarm_trigger": "",
            "teleop_mode": None,
            "link_ok": True,
            "link": {"last_message_age_sec": 0.0, "reconnects": 0},
            "counters": {},
            "blacklist": [],
            "battery": {
                "status": "unavailable",
                "reason": "NO_SENSOR_INSTALLED",
                "percent": None,
            },
            "octopus_transform": None,
            "simulated": True,
        }


async def run(host, port, speed, collect, dialect):
    robot = Robot(speed, collect, dialect)
    url = f"ws://{host}:{port}"
    print(f"connecting to {url}")

    handlers = {
        "/octopus/fake_eve_gps_start": robot.on_datum,
        "/octopus/trash_goal": robot.on_goal,
        "/octopus/trash_gps": robot.on_trash_gps,
    }
    types = {
        "/octopus/fake_eve_gps_start": "sensor_msgs/NavSatFix",
        "/octopus/trash_goal": "sensor_msgs/NavSatFix",
        "/octopus/trash_gps": "std_msgs/String",
    }

    async with connect(url, max_size=2 ** 20) as ws:
        print("connected - publishing status at 2 Hz, Ctrl-C to stop\n")

        for topic, msg_type in types.items():
            await ws.send(json.dumps({"op": "subscribe", "id": f"sub:{topic}",
                                      "topic": topic, "type": msg_type}))
        for topic in (STATUS_TOPIC, GOAL_DONE_TOPIC):
            await ws.send(json.dumps({"op": "advertise", "id": f"adv:{topic}",
                                      "topic": topic, "type": "std_msgs/String"}))

        async def receive():
            while True:
                frame = json.loads(await ws.recv())
                if frame.get("op") == "publish":
                    handler = handlers.get(frame.get("topic"))
                    if handler:
                        handler(frame.get("msg") or {})
                elif frame.get("op") == "status" and frame.get("level") in ("error", "warning"):
                    print(f"rosbridge [{frame['level']}] {frame.get('msg')}")

        async def transmit():
            last_print = 0.0
            while True:
                done_id = robot.step()
                if done_id is not None:
                    await ws.send(json.dumps({
                        "op": "publish", "id": "pub:done",
                        "topic": GOAL_DONE_TOPIC,
                        "msg": {"data": str(done_id)},
                    }))

                payload = robot.status_payload()
                await ws.send(json.dumps({
                    "op": "publish", "id": "pub:status",
                    "topic": STATUS_TOPIC,
                    "msg": {"data": json.dumps(payload)},
                }))

                now = time.monotonic()
                if now - last_print >= 2.0:
                    last_print = now
                    remaining = robot.distance_remaining
                    where = (f"{robot.lat:.7f}, {robot.lon:.7f}"
                             if robot.lat is not None else "no datum yet")
                    print(f"  {robot.nav_status:<11} {where}"
                          + (f"  {remaining:.2f} m to go" if remaining is not None else ""))

                await asyncio.sleep(STATUS_PERIOD_SEC)

        await asyncio.gather(receive(), transmit())


def main():
    parser = argparse.ArgumentParser(description="Simulated GripperX over rosbridge")
    parser.add_argument("--host", default="127.0.0.1", help="Octopus host running rosbridge")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--speed", type=float, default=0.4, help="m/s, default 0.4")
    parser.add_argument(
        "--dialect",
        choices=("dashboard", "gripperx"),
        default="dashboard",
        help="Wire shape for the status payload. 'dashboard' (default) is what "
             "this script has always sent and what the dashboard reads directly. "
             "'gripperx' is what the real robot sends, and exercises the "
             "translation in device_status_backend_bridge_node.",
    )
    parser.add_argument(
        "--no-collect", dest="collect", action="store_false",
        help="drive to the goal but never publish trash_goal_done",
    )
    args = parser.parse_args()

    try:
        asyncio.run(run(args.host, args.port, args.speed, args.collect, args.dialect))
    except KeyboardInterrupt:
        print("\nstopped")
    except OSError as exc:
        print(f"connection to {args.host}:{args.port} failed: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
