#!/usr/bin/env python3
"""Offline A/B replay of the EKF yaw-rate fusion (sub-issue 2.5, LIDAR/IMU issue draft).

One command per step; `matrix` chains them for a plan in config/imu_replay_plan.yaml.

    gripperx_imu_replay.sh matrix --stage drive_core    (wrapper in Software/ros2/scripts)
    gripperx_imu_replay.sh report                       (tables, imu_replay_report.py)

WHAT A RUN IS. A bag stripped of everything the old stack computed is played in real
time into localization_input_node, ekf_node and slam_toolbox (launch/imu_replay
.launch.py), with one override file per variant, and everything is recorded into a
new bag that yaw_compare reads as it reads a robot bag.

ISOLATION, THE ONE HARD RULE. The bags contain /cmd_vel and the robot may be powered
on the same network, listening on domain 20. Every child process gets the environment
of isolated_env(): domain 87, localhost only, whatever the calling shell has set. An
inherited or requested domain 20 is refused, not corrected: somebody who has 20 in
this shell is one mistyped command away from the robot.

WHY THE BAG IS REWRITTEN (prepare), rather than filtered at playback:
  - `ros2 bag play` filters whole topics, but /tf mixes frames to keep (wheel joints,
    arm) with the two frames the old stack computed (odom->base_footprint, map->odom).
    A surviving old transform beside the new one is a silent failure.
  - /tf_static is one latched message at bag start. It is repeated at 1 Hz here,
    because ros2 bag record --use-sim-time writes nothing before its first /clock and
    yaw_compare needs the mounts from the recorded bag.
Bag receive time is the sim clock. That is sound only because the laptop and Pi clocks
agree: receive minus stamp is +5 ms median on /imu/bno085/data and /joint_states
(drive15), so stamps are never ahead of sim time. check_clock_offset asserts this for
each input bag.

--clock 200, not the 40 Hz default: ekf_node's 50 Hz timer runs on sim time and could
otherwise only fire on /clock ticks, 25 ms apart.

--delay: without it the first 3 s of output were missing, differently per run (EKF
output starting 3 s late, or the recorder), because reliable volatile topics and
/clock are lost until the player's publishers are discovered. Measured on the smoke run.

check_run() FAILS LOUDLY on a frozen, partial or duplicated run; a run that fails it
never reaches a table.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPLAY_DOMAIN_ID = 87
FORBIDDEN_DOMAIN_ID = 20        # the robot
PLAY_CLOCK_HZ = 200
PLAY_DELAY_S = 6     # discovery of the player's publishers before the first message
# What the replay needs from the bag. Everything else, above all /odometry/filtered,
# /wheel/odom and /imu/data/filtered, is regenerated or irrelevant.
KEEP_TOPICS = ("/scan", "/joint_states", "/imu/bno085/data", "/cmd_vel", "/tf", "/tf_static",
               "/laser/odom")
REGENERATED_TF = (("odom", "base_footprint"), ("map", "odom"))
RECORD_TOPICS = KEEP_TOPICS + ("/wheel/odom", "/odometry/filtered", "/imu/data/filtered")
ENV_KEYS_FORCED = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE")
STACK_NODES = {"ekf": "ekf_node", "slam": "async_slam_toolbox_node",
               "input": "localization_input_node"}
DEFAULT_OUT = "~/gripperx_bags/replay_2p5"
MAX_START_LOSS_S = 8.0
# CHOSEN margins, not derived: each is a loud-failure limit set well outside what the
# healthy runs show.
MAX_EKF_GAP_S = 1.0          # /wheel/odom keeps the EKF publishing at rest too
MAX_EKF_FREEZE_S = 1.0       # s of TURNING with no change of the EKF yaw value
MAX_SLAM_STALL_S = 6.0       # s of motion with no map->odom change; worst healthy 2.4
SLAM_STAMP_LEAD_S = 0.5      # a map->odom stamp is about this far ahead of its scan
MIN_SLAM_UPDATES_PER_MIN = 20   # lowest healthy moving run: 33.75


class ReplayError(RuntimeError):
    """A run or input that must not reach a table."""


# --------------------------------------------------------------------------- safety

def isolated_env(base=None, domain: int = REPLAY_DOMAIN_ID) -> dict:
    """Environment for every child. Refuses domain 20, inherited or requested."""
    env = dict(os.environ if base is None else base)
    if str(env.get("ROS_DOMAIN_ID", "")).strip() == str(FORBIDDEN_DOMAIN_ID):
        raise ReplayError(f"ROS_DOMAIN_ID={FORBIDDEN_DOMAIN_ID} (the robot) in this shell: refused. "
                          "Unset it and run again; the replay sets its own domain.")
    if int(domain) == FORBIDDEN_DOMAIN_ID:
        raise ReplayError(f"domain {FORBIDDEN_DOMAIN_ID} is the robot: refused")
    env["ROS_DOMAIN_ID"] = str(int(domain))
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    return env


# ---------------------------------------------------------------------- bag rewriting

def _reader(uri, topics=None):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=""),
           rosbag2_py.ConverterOptions("", ""))
    if topics is not None:
        r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return r


def bag_span(uri):
    """-> (start_s, duration_s) from the bag metadata."""
    m = _reader(uri).get_metadata()
    return m.starting_time.nanoseconds * 1e-9, m.duration.nanoseconds * 1e-9


def tf_keep(frame_id: str, child: str) -> bool:
    return (frame_id, child) not in REGENERATED_TF


class GyroInjector:
    """Fault on the raw gyro stream, the INPUT of localization_input_node (2.5b, D14).

    Injected there and not on /imu/data/filtered, so the plausibility check, the bias
    estimator and the EKF all see the fault as they would see a sick sensor. Stamps
    keep running, as with a driver republishing a latched report.

    start and duration in s after bag start (as drop_after), on the receive time.
      stuck   every sample repeats the measurement of the last one before start
      zero    angular velocity 0, the rest of the sample untouched
      silent  samples dropped; the topic returns at start + duration
    """
    MODES = ("stuck", "zero", "silent")

    def __init__(self, mode, start, duration, t0, check_tol=None, recovers=True,
                 max_slam_stall_s=None):
        if mode not in self.MODES:
            raise ReplayError(f"unknown injection {mode}")
        if duration <= 0.0:
            raise ReplayError("an injection is bounded: duration > 0")
        self.mode = mode
        self.lo = int((t0 + start) * 1e9)
        self.hi = int((t0 + start + duration) * 1e9)
        self.frozen = None
        self.n_changed = 0

    def apply(self, msg, t_ns):
        """-> the message to write, or None to drop it. Keeps the last clean sample."""
        if t_ns < self.lo:
            self.frozen = msg
            return msg
        if t_ns >= self.hi:
            return msg
        self.n_changed += 1
        if self.mode == "silent":
            return None
        if self.mode == "zero":
            msg.angular_velocity.x = msg.angular_velocity.y = msg.angular_velocity.z = 0.0
            return msg
        if self.frozen is None:
            raise ReplayError("stuck injection before the first sample")
        msg.angular_velocity = self.frozen.angular_velocity
        msg.orientation = self.frozen.orientation
        msg.linear_acceleration = self.frozen.linear_acceleration
        return msg


def prepare(src, dst, start=None, duration=None, drop_topic=None, drop_after=None,
            scan_lead_size=None, scan_lead_sec=5.0, scan_stamp_shift=None,
            gyro_inject=None) -> dict:
    """Write the stripped input bag. Times in s after bag start. -> counts.

    scan_lead_size: during the first scan_lead_sec, keep only scans of this many
    ranges. slam_toolbox rejects every scan whose size differs from the FIRST one it
    processes, and the LD06 alternates 453/454, so that first scan decides which half
    of all scans it sees. Which scan is first depends on when the EKF's first
    transform arrives, i.e. on the variant: an uncontrolled A/B difference. The robot
    idles through this lead-in, so no motion is lost.

    scan_stamp_shift: seconds added to every /scan header stamp. An experiment for
    2.7 (is slam's arc result a question of which pose a sweep is paired with?), not
    a correction; the receive time stays as it was.

    gyro_inject: {mode, start, duration}, see GyroInjector.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_msgs.msg import TFMessage

    dst = str(dst)
    if os.path.exists(dst):
        raise ReplayError(f"{dst} exists; prepare never overwrites")
    t0, _ = bag_span(src)
    r_static = _reader(src, ["/tf_static"])
    static = r_static.read_next()[1] if r_static.has_next() else None
    if static is None:
        raise ReplayError(f"{src}: no /tf_static")

    r = _reader(src, KEEP_TOPICS)
    types = {t.name: t for t in r.get_all_topics_and_types()}
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name in KEEP_TOPICS:
        if name in types:
            w.create_topic(types[name])
    lo = int((t0 + (start or 0.0)) * 1e9)
    hi = int((t0 + (start or 0.0) + duration) * 1e9) if duration else None
    drop_ns = int((t0 + drop_after) * 1e9) if drop_after is not None else None
    lead_ns = lo + int(scan_lead_sec * 1e9) if scan_lead_size is not None else None
    counts, next_static = {}, lo
    removed_tf = 0
    injector = GyroInjector(t0=t0, **gyro_inject) if gyro_inject else None
    while r.has_next():
        topic, raw, t = r.read_next()
        if t < lo:
            continue
        if hi is not None and t > hi:
            break
        while next_static <= t:
            w.write("/tf_static", static, next_static)
            next_static += 1_000_000_000
        if topic == "/tf_static":
            continue
        if topic == drop_topic and drop_ns is not None and t >= drop_ns:
            continue
        if topic == "/imu/bno085/data" and injector is not None:
            msg = injector.apply(deserialize_message(raw, Imu), t)
            if msg is None:
                continue
            raw = serialize_message(msg)
        if topic == "/scan" and lead_ns is not None and t < lead_ns:
            if len(deserialize_message(raw, LaserScan).ranges) != scan_lead_size:
                continue
        if topic == "/scan" and scan_stamp_shift:
            msg = deserialize_message(raw, LaserScan)
            ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec + \
                int(round(scan_stamp_shift * 1e9))
            msg.header.stamp.sec, msg.header.stamp.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
            raw = serialize_message(msg)
        if topic == "/tf":
            msg = deserialize_message(raw, TFMessage)
            kept = [tr for tr in msg.transforms if tf_keep(tr.header.frame_id, tr.child_frame_id)]
            removed_tf += len(msg.transforms) - len(kept)
            if not kept:
                continue
            if len(kept) != len(msg.transforms):
                msg.transforms = kept
                raw = serialize_message(msg)
        w.write(topic, raw, t)
        counts[topic] = counts.get(topic, 0) + 1
    del w
    if injector is not None and injector.n_changed == 0:
        raise ReplayError(f"{dst}: injection touched no sample")
    return {"counts": counts, "removed_tf": removed_tf, "t0": t0,
            "injected_samples": injector.n_changed if injector else 0}


def _shift_header(header, ns):
    t = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec + ns
    header.stamp.sec, header.stamp.nanosec = t // 1_000_000_000, t % 1_000_000_000


def prepare_chain(first, second, dst, gap_s) -> dict:
    """Two prepared bags as one, second moved to start gap_s after first ends (2.7).

    Tests the pre-existing-map reading of the 1.4 rad/s mismatch: one slam session over
    both. Valid only if the robot stood at the same pose at the seam; that is a
    property of the recordings and measured in the report, not checked here. gap_s
    must exceed reset_gap_sec of localization_input_node: the seam then goes through the
    designed wheel position reset instead of a position jump.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message

    dst = str(dst)
    if os.path.exists(dst):
        raise ReplayError(f"{dst} exists; prepare never overwrites")
    t1, d1 = bag_span(first)
    t2, _ = bag_span(second)
    shift_ns = int(round((t1 + d1 + gap_s - t2) * 1e9))
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    r = _reader(first)
    tops = {t.name: t for t in r.get_all_topics_and_types()}
    for t in tops.values():
        w.create_topic(t)
    counts = {}
    while r.has_next():
        topic, raw, t = r.read_next()
        w.write(topic, raw, t)
        counts[topic] = counts.get(topic, 0) + 1
    r = _reader(second)
    cls = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, raw, t = r.read_next()
        msg = deserialize_message(raw, cls[topic])
        if hasattr(msg, "transforms"):
            for tr in msg.transforms:
                _shift_header(tr.header, shift_ns)
        elif hasattr(msg, "header"):
            _shift_header(msg.header, shift_ns)
        w.write(topic, serialize_message(msg), t + shift_ns)
        counts[topic] = counts.get(topic, 0) + 1
    del w
    return {"counts": counts, "shift_s": shift_ns * 1e-9, "second_starts_s": d1 + gap_s}


def check_prepared(uri) -> dict:
    """Re-read the written bag: no old-stack output may have survived."""
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    r = _reader(uri)
    names = {t.name for t in r.get_all_topics_and_types()}
    extra = names - set(KEEP_TOPICS)
    if extra:
        raise ReplayError(f"{uri}: unexpected topics {sorted(extra)}")
    r.set_filter(__import__("rosbag2_py").StorageFilter(topics=["/tf"]))
    n, pairs = 0, set()
    while r.has_next():
        _, raw, _ = r.read_next()
        for tr in deserialize_message(raw, TFMessage).transforms:
            n += 1
            pairs.add((tr.header.frame_id, tr.child_frame_id))
    bad = pairs & set(REGENERATED_TF)
    if bad:
        raise ReplayError(f"{uri}: old-stack transforms survived: {sorted(bad)}")
    return {"tf_transforms": n, "tf_pairs": len(pairs)}


def check_clock_offset(uri, max_ahead_s=0.02) -> dict:
    """The sim clock is receive time; a stamp ahead of it would stall the EKF queue."""
    import numpy as np
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    r = _reader(uri, ["/imu/bno085/data"])
    lag = []
    while r.has_next() and len(lag) < 5000:
        _, raw, t = r.read_next()
        m = deserialize_message(raw, Imu)
        lag.append(t * 1e-9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    med = float(np.median(lag)) if lag else float("nan")
    if not (abs(med) < max_ahead_s):
        raise ReplayError(f"{uri}: IMU receive minus stamp median {med:+.3f} s; the bag's "
                          "receive time cannot be the sim clock without a shift")
    return {"imu_recv_minus_stamp_median_s": med}


def prepared_bag(out_root, label, src, **kw) -> Path:
    dst = Path(out_root) / "inputs" / label
    if not dst.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        check_clock_offset(src)
        info = prepare(src, dst, **kw)
        info["check"] = check_prepared(dst)
        (dst.parent / f"{label}.prepare.json").write_text(json.dumps(info, indent=1))
    return dst


def prepared_input(out_root, plan, key) -> Path:
    """One plan input as a bag; a chain builds its two parts first."""
    spec = plan["inputs"][key]
    if "chain" not in spec:
        return prepared_bag(out_root, key, Path(spec["bag"]).expanduser(),
                            **spec.get("prepare", {}))
    dst = Path(out_root) / "inputs" / key
    if not dst.exists():
        ch = spec["chain"]
        a, b = (prepared_input(out_root, plan, ch[k]) for k in ("first", "second"))
        info = prepare_chain(a, b, dst, ch["gap_s"])
        info["check"] = check_prepared(dst)
        (dst.parent / f"{key}.prepare.json").write_text(json.dumps(info, indent=1))
    return dst


# ------------------------------------------------------------------- plan and overrides

def plan_path() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("gripperx_localization")) / "config" / \
        "imu_replay_plan.yaml"


def load_plan(path=None) -> dict:
    import yaml
    with open(path or plan_path()) as f:
        return yaml.safe_load(f)


def build_override(variant: dict, base_q=None) -> dict:
    """Per-node parameter override for one variant. Pure, tested without ROS."""
    ekf, inp, slam = {}, {}, {}
    if variant.get("imu0", True) is False:
        ekf["imu0"] = ""
        ekf["imu0_config"] = [False] * 15
    if "imu_yaw_rate_variance" in variant:
        inp["imu_yaw_rate_variance"] = float(variant["imu_yaw_rate_variance"])
    if "wheel_odom_yaw_rate_variance" in variant:
        inp["wheel_odom_yaw_rate_variance"] = float(variant["wheel_odom_yaw_rate_variance"])
    for key, idx in (("q_vyaw", 11), ("q_yaw", 5)):
        if key in variant:
            if base_q is None:
                raise ValueError("q_* needs the base process noise")
            base_q = list(base_q)
            base_q[idx] = float(variant[key])
            ekf["process_noise_covariance"] = base_q
    if "imu0_twist_rejection_threshold" in variant:
        ekf["imu0_twist_rejection_threshold"] = float(variant["imu0_twist_rejection_threshold"])
    if "angle_variance_penalty" in variant:
        slam["angle_variance_penalty"] = float(variant["angle_variance_penalty"])
    if variant.get("ekf_debug_file"):
        ekf["debug"] = True
        ekf["debug_out_file"] = str(variant["ekf_debug_file"])
    out = {}
    if ekf:
        out["ekf_filter_node"] = {"ros__parameters": ekf}
    if inp:
        out["localization_input_node"] = {"ros__parameters": inp}
    if slam:
        out["slam_toolbox"] = {"ros__parameters": slam}
    # ROS rejects an empty parameter file.
    return out or {"localization_input_node": {"ros__parameters": {"use_sim_time": True}}}


def active_config() -> dict:
    """config/localization.yaml as installed, the file every run layers its override on."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    p = Path(get_package_share_directory("gripperx_localization")) / "config" / "localization.yaml"
    return yaml.safe_load(p.read_text())


def base_process_noise() -> list:
    return active_config()["ekf_filter_node"]["ros__parameters"]["process_noise_covariance"]


# ------------------------------------------------------------------------------- run

def _pid_table(launch_pid) -> dict:
    import psutil
    out = {}
    for ch in psutil.Process(launch_pid).children(recursive=True):
        try:
            cmd = " ".join(ch.cmdline())
        except psutil.Error:
            continue
        for key, needle in STACK_NODES.items():
            if needle in cmd and "ros2 launch" not in cmd:
                if key in out:
                    raise ReplayError(f"two {needle} processes in one stack")
                out[key] = ch
    return out


def _cpu_times(procs) -> dict:
    res = {}
    for k, p in procs.items():
        t = p.cpu_times()
        res[k] = t.user + t.system
    return res


def run_once(out_root, name, input_bag, variant, env) -> Path:
    """One replay. Returns the run directory; raises ReplayError if it failed its checks."""
    import yaml
    run_dir = Path(out_root) / "runs" / name
    if run_dir.exists():
        raise ReplayError(f"{run_dir} exists; a run is never overwritten")
    run_dir.mkdir(parents=True)
    variant = dict(variant)
    if variant.pop("ekf_debug", False):
        variant["ekf_debug_file"] = str(run_dir / "ekf_debug.txt")
    override = run_dir / "override.yaml"
    override.write_text(yaml.safe_dump(build_override(variant, base_process_noise())))
    meta = {"name": name, "input": str(input_bag), "variant": variant,
            "domain": env["ROS_DOMAIN_ID"], "localhost_only": env["ROS_LOCALHOST_ONLY"],
            "discovery_range": env["ROS_AUTOMATIC_DISCOVERY_RANGE"],
            "git": subprocess.run(["git", "rev-parse", "--short", "HEAD"], text=True,
                                  capture_output=True, cwd=Path(__file__).parent).stdout.strip(),
            "loadavg_start": os.getloadavg(), "started": time.strftime("%F %T")}

    log = open(run_dir / "stack.log", "w")
    stack = subprocess.Popen(["ros2", "launch", "gripperx_localization", "imu_replay.launch.py",
                              f"override_params:={override}"], stdout=log, stderr=subprocess.STDOUT,
                             env=env, start_new_session=True)
    rec = play = None
    try:
        deadline = time.monotonic() + 30
        procs = {}
        while time.monotonic() < deadline:
            procs = _pid_table(stack.pid)
            if len(procs) == len(STACK_NODES):
                break
            time.sleep(0.5)
        if len(procs) != len(STACK_NODES):
            raise ReplayError(f"stack did not start: {sorted(procs)}")
        time.sleep(5)
        rec = subprocess.Popen(
            ["ros2", "bag", "record", "--use-sim-time", "-s", "mcap", "-o", str(run_dir / "out"),
             "--topics", *RECORD_TOPICS], stdout=open(run_dir / "record.log", "w"),
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        time.sleep(4)
        cpu0, wall0 = _cpu_times(procs), time.monotonic()
        play = subprocess.Popen(
            ["ros2", "bag", "play", str(input_bag), "--clock", str(PLAY_CLOCK_HZ), "--rate", "1.0",
             "--delay", str(PLAY_DELAY_S), "--disable-keyboard-controls"], stdout=open(run_dir / "play.log", "w"),
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        time.sleep(PLAY_DELAY_S + 8)
        probe = subprocess.run([sys.executable, "-m", "gripperx_localization.imu_replay", "probe",
                                "--out", str(run_dir / "probe.json")], env=env, timeout=60,
                               capture_output=True, text=True)
        if probe.returncode:
            raise ReplayError(f"probe failed: {probe.stderr[-500:]}")
        if play.wait() != 0:
            raise ReplayError("ros2 bag play failed")
        wall = time.monotonic() - wall0
        cpu1 = _cpu_times(procs)
        time.sleep(2)
        meta["cpu_core_avg"] = {k: (cpu1[k] - cpu0[k]) / wall for k in cpu0}
        meta["cpu_wall_s"] = wall
        meta["rss_mb"] = {k: p.memory_info().rss / 1e6 for k, p in procs.items()}
        meta["loadavg_end"] = os.getloadavg()
    finally:
        for p in (play, rec, stack):
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in (play, rec, stack):
            if p is not None:
                try:
                    p.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait()
        log.close()
        (run_dir / "meta.json").write_text(json.dumps(meta, indent=1))
    result = check_run(run_dir, input_bag, variant)
    (run_dir / "check.json").write_text(json.dumps(result, indent=1))
    return run_dir


def graph_complete(graph) -> bool:
    """Every endpoint a healthy stack must show. A wrong graph stays a check_run failure."""
    names = [n for g in graph.values() for lst in g.values() for n in lst]
    return ("_NODE_NAME_UNKNOWN_" not in names
            and {"ekf_filter_node", "slam_toolbox", "rosbag2_player"} <= set(graph["/tf"]["pub"])
            and "localization_input_node" in graph["/wheel/odom"]["pub"]
            and "ekf_filter_node" in graph["/wheel/odom"]["sub"]
            and "ekf_filter_node" in graph["/odometry/filtered"]["pub"]
            and "localization_input_node" in graph["/imu/data/filtered"]["pub"])


def probe_main(out_path, timeout=30.0):
    """Inside the replay domain: who publishes what, and the parameters in force."""
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient
    rclpy.init()
    node = rclpy.create_node("imu_replay_probe")
    # Discovery of names is asynchronous: a snapshot taken too early misses endpoints or
    # reads _NODE_NAME_UNKNOWN_. Wait for every endpoint graph_complete names.
    graph = {}
    deadline = time.monotonic() + timeout
    while True:
        rclpy.spin_once(node, timeout_sec=0.2)
        graph = {}
        for topic in ("/tf", "/wheel/odom", "/odometry/filtered", "/imu/data/filtered"):
            graph[topic] = {
                "pub": sorted(i.node_name for i in node.get_publishers_info_by_topic(topic)),
                "sub": sorted(i.node_name for i in node.get_subscriptions_info_by_topic(topic))}
        if graph_complete(graph) or time.monotonic() > deadline:
            break
        time.sleep(0.5)
    params = {}
    for target, names in (
            ("ekf_filter_node", ["imu0", "odom0", "odom1", "process_noise_covariance",
                                 "imu0_twist_rejection_threshold", "use_sim_time", "publish_tf"]),
            ("localization_input_node", ["imu_yaw_rate_variance", "wheel_odom_yaw_rate_variance",
                                         "imu_input_topic", "imu_output_topic", "publish_tf",
                                         "use_sim_time"]),
            ("slam_toolbox", ["angle_variance_penalty", "use_sim_time"])):
        cli = AsyncParameterClient(node, target)
        if not cli.wait_for_services(timeout_sec=timeout):
            raise SystemExit(f"no parameter service on {target}")
        fut = cli.get_parameters(names)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        vals = fut.result().values if fut.result() else []
        from rclpy.parameter import parameter_value_to_python
        params[target] = {n: parameter_value_to_python(v) for n, v in zip(names, vals)}
    # Same race for the EKF's subscription, which only the parameters can demand.
    if params["ekf_filter_node"]["imu0"]:
        deadline = time.monotonic() + timeout
        topic = "/imu/data/filtered"
        while time.monotonic() < deadline:
            graph[topic]["sub"] = sorted(
                i.node_name for i in node.get_subscriptions_info_by_topic(topic))
            if "ekf_filter_node" in graph[topic]["sub"]:
                break
            rclpy.spin_once(node, timeout_sec=0.5)
    Path(out_path).write_text(json.dumps({"graph": graph, "params": params}, indent=1, default=list))
    node.destroy_node()
    rclpy.shutdown()


def tf_pair_stats(uri, pairs=REGENERATED_TF) -> dict:
    """Per pair: count, stamp span and largest hole, stamps not newer, and when the value changed."""
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    r = _reader(uri, ["/tf"])
    st = {p: {"n": 0, "first": None, "last": None, "not_newer": 0, "older": 0, "max_gap": 0.0,
              "changes": [], "yaw_changes": [], "value": None} for p in pairs}
    while r.has_next():
        _, raw, _ = r.read_next()
        for tr in deserialize_message(raw, TFMessage).transforms:
            p = (tr.header.frame_id, tr.child_frame_id)
            if p not in st:
                continue
            s = st[p]
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            s["n"] += 1
            if s["last"] is not None:
                s["not_newer"] += t <= s["last"]
                s["older"] += t < s["last"]
                s["max_gap"] = max(s["max_gap"], t - s["last"])
            s["first"] = t if s["first"] is None else s["first"]
            s["last"] = t
            q, tt = tr.transform.rotation, tr.transform.translation
            value = (q.z, q.w, tt.x, tt.y)
            if s["value"] is not None and value != s["value"]:
                s["changes"].append(t)
                if value[:2] != s["value"][:2]:
                    s["yaw_changes"].append(t)
            s["value"] = value
    return {f"{a}->{b}": {k: v for k, v in s.items() if k != "value"} for (a, b), s in st.items()}


def stamps_of(uri, topic, type_name) -> list:
    """Header stamps of one topic, epoch seconds."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    cls = get_message(type_name)
    r, out = _reader(uri, [topic]), []
    while r.has_next():
        _, raw, _ = r.read_next()
        h = deserialize_message(raw, cls).header
        out.append(h.stamp.sec + h.stamp.nanosec * 1e-9)
    return out


def longest_motion_without_update(wheel_uri, changes, t_still=0.2, v_still=0.05,
                                  lead=SLAM_STAMP_LEAD_S) -> float:
    """Most seconds of robot MOTION between two value changes. slam is idle at rest by
    design, so wall-time gaps say nothing; motion time without an update does."""
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    r, worst, run, last, k = _reader(wheel_uri, ["/wheel/odom"]), 0.0, 0.0, None, 0
    changes = sorted(changes)
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        while k < len(changes) and changes[k] - lead <= t:
            k, run = k + 1, 0.0
        tw = m.twist.twist
        if last is not None and (abs(tw.angular.z) > t_still
                                 or math.hypot(tw.linear.x, tw.linear.y) > v_still):
            run += min(t - last, 0.5)
            worst = max(worst, run)
        last = t
    return worst


def _meta_counts(uri) -> dict:
    return {t.topic_metadata.name: t.message_count
            for t in _reader(uri).get_metadata().topics_with_message_count}


def imu_series(uri, topic):
    """-> (stamps s, raw z) of one Imu topic, as numpy arrays in stamp order."""
    import numpy as np
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    r, rows = _reader(uri, [topic]), []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Imu)
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z))
    a = np.array(rows)
    return a[:, 0], a[:, 1]


def injection_faults(out_bag, inj, altered_min=0.1, tol=0.01, margin_s=0.1) -> list:
    """Did the fault reach what the EKF consumes? Reads the RECORDED /imu/data/filtered,
    so a variant whose override quietly did nothing cannot pass. -> failures.

    inj: the plan's gyro_inject, plus clean_bag (the stream without the fault) and
    t0_source (bag start of clean_bag; injection times count from it).
    """
    import numpy as np
    t_out, z_out = imu_series(out_bag, "/imu/data/filtered")
    t_cln, z_cln = imu_series(inj["clean_bag"], "/imu/bno085/data")
    lo, hi = inj["t0_source"] + inj["start"], inj["t0_source"] + inj["start"] + inj["duration"]
    fails = []
    inside = (t_out >= lo + margin_s) & (t_out < hi - margin_s)
    if inj["mode"] == "silent":
        if inside.any():
            fails.append(f"{int(inside.sum())} samples inside the silence")
    else:
        if not inside.any():
            return [f"no /imu/data/filtered sample inside the {inj['mode']} window"]
        z = z_out[inside]
        spread = float(z.max() - z.min())
        if inj["mode"] == "zero" and float(np.abs(z).max()) > tol:
            fails.append(f"zero injection: |z| up to {np.abs(z).max():.3f} in the output")
        if inj["mode"] == "stuck" and spread > inj.get("check_tol", tol):
            fails.append(f"stuck injection: output spread {spread:.3f}")
        moved = float(np.abs(z - np.interp(t_out[inside], t_cln, z_cln)).max())
        if moved < altered_min:
            fails.append(f"injection altered the EKF input by only {moved:.3f} rad/s")
    # The stream must be back, or "recovery" measures nothing.
    after = t_out >= hi + 0.5
    if inj.get("recovers", True):
        if not after.any():
            fails.append("no IMU output after the injection")
        else:
            late = np.abs(z_out[after] - np.interp(t_out[after], t_cln, z_cln))
            if float(np.percentile(late, 50)) > inj.get("check_tol", tol):
                fails.append(f"output off the clean stream after the end, median {np.median(late):.3f}")
    return fails


def check_run(run_dir, input_bag, variant, min_cover=0.97) -> dict:
    """Everything that must hold before a run's numbers may be used. Raises ReplayError."""
    run_dir = Path(run_dir)
    out = run_dir / "out"
    t0, dur = bag_span(input_bag)
    t_out, dur_out = bag_span(out)
    cin, cout = _meta_counts(input_bag), _meta_counts(out)
    tf = tf_pair_stats(out)
    probe = json.loads((run_dir / "probe.json").read_text())
    fuses = variant.get("imu0", True) is not False
    silent = variant.get("expect_silent_imu", False)
    standstill = variant.get("expect_standstill", False)
    fails = []

    def need(cond, msg):
        if not cond:
            fails.append(msg)

    need(dur_out >= min_cover * dur, f"output covers {dur_out:.1f} s of {dur:.1f} s")
    for k in ("/wheel/odom", "/odometry/filtered"):
        need(cout.get(k, 0) >= 15 * min_cover * dur, f"{k}: {cout.get(k, 0)} messages in {dur:.0f} s")
    if not silent:
        need(cout.get("/imu/data/filtered", 0) >= min_cover * cin.get("/imu/bno085/data", 0),
             f"/imu/data/filtered {cout.get('/imu/data/filtered', 0)} of "
             f"{cin.get('/imu/bno085/data', 0)}")
    need(cout.get("/scan", 0) >= min_cover * cin.get("/scan", 0),
         f"/scan {cout.get('/scan', 0)} of {cin.get('/scan', 0)}")
    need(cout.get("/tf_static", 0) > 0, "no /tf_static in the output")
    silent_after = variant.get("silent_after")
    if silent:
        # A dropped-topic run is worthless if the IMU in fact kept arriving.
        need(silent_after is not None, "silent run without silent_after")
        if silent_after is not None:
            limit = t0 + silent_after + 0.1
            for bag, topic in ((input_bag, "/imu/bno085/data"), (out, "/imu/data/filtered")):
                stamps = stamps_of(bag, topic, "sensor_msgs/msg/Imu")
                need(stamps and max(stamps) <= limit, f"{topic} not silent after {silent_after} s")
                need(len(stamps) >= 40 * min_cover * silent_after,
                     f"{topic}: {len(stamps)} samples before going silent")
    for pair, s in tf.items():
        need(s["n"] > 100, f"{pair}: {s['n']} transforms")
        if s["first"] is not None:
            need(s["last"] - s["first"] >= dur - MAX_START_LOSS_S,
                 f"{pair}: stamps span {s['last'] - s['first']:.1f} s of {dur:.1f} s")
            need(len(s["changes"]) > 0 or standstill, f"{pair}: frozen")
            need(s["older"] <= 0.01 * s["n"],
                 f"{pair}: {s['older']} of {s['n']} stamps older than their predecessor "
                 "(second publisher?)")
    # slam_toolbox repeats a map->odom stamp between scans by design; the EKF never does.
    need(tf["odom->base_footprint"]["not_newer"] == 0, "odom->base stamps repeat: second EKF?")
    need(tf["odom->base_footprint"]["max_gap"] <= variant.get("max_ekf_gap_s", MAX_EKF_GAP_S),
         f"EKF silent for {tf['odom->base_footprint']['max_gap']:.1f} s inside the run")
    freeze = longest_motion_without_update(out, tf["odom->base_footprint"]["yaw_changes"],
                                           v_still=float("inf"), lead=0.0)
    need(freeze <= MAX_EKF_FREEZE_S or standstill, f"EKF yaw frozen for {freeze:.1f} s of turning")
    # The graph probe is one snapshot; a stray publisher later on shows only in the data.
    wheel = stamps_of(out, "/wheel/odom", "nav_msgs/msg/Odometry")
    back = sum(b < a for a, b in zip(wheel, wheel[1:]))
    need(back == 0, f"/wheel/odom stamps go back {back} times: second publisher?")
    need(len(wheel) <= cin.get("/joint_states", 0),
         f"/wheel/odom {len(wheel)} messages from {cin.get('/joint_states', 0)} /joint_states")
    slam_changes = tf["map->odom"]["changes"]
    stall = longest_motion_without_update(out, slam_changes)
    # An injection may blind slam on purpose (odometry shows no turn, slam skips the scans);
    # such a plan entry states the idle time it expects, and it is still a limit.
    stall_limit = variant.get("injection", {}).get("max_slam_stall_s") or MAX_SLAM_STALL_S
    need(stall <= stall_limit or standstill, f"slam idle for {stall:.1f} s of motion")
    need(standstill or len(slam_changes) >= MIN_SLAM_UPDATES_PER_MIN * dur / 60.0,
         f"slam made {len(slam_changes)} updates in {dur:.0f} s")
    ekf_tf = tf["odom->base_footprint"]["n"]
    need(abs(ekf_tf - cout.get("/odometry/filtered", 0)) <= 0.01 * ekf_tf,
         f"odom->base {ekf_tf} vs /odometry/filtered {cout.get('/odometry/filtered', 0)}: "
         "not one publisher")
    g = probe["graph"]
    need(g["/wheel/odom"]["pub"] == ["localization_input_node"], f"/wheel/odom pub {g['/wheel/odom']['pub']}")
    need(g["/odometry/filtered"]["pub"] == ["ekf_filter_node"],
         f"/odometry/filtered pub {g['/odometry/filtered']['pub']}")
    tf_pub = g["/tf"]["pub"]
    # slam_toolbox holds two /tf writers in one process (seen on every run); _pid_table
    # guarantees that it is one process.
    need(tf_pub.count("ekf_filter_node") == 1 and tf_pub.count("slam_toolbox") >= 1
         and set(tf_pub) <= {"ekf_filter_node", "slam_toolbox", "rosbag2_player"},
         f"/tf publishers {tf_pub}")
    need(("ekf_filter_node" in g["/imu/data/filtered"]["sub"]) == fuses,
         f"EKF on /imu/data/filtered: {g['/imu/data/filtered']['sub']}, imu0 wanted {fuses}")
    p = probe["params"]
    need(p["localization_input_node"]["publish_tf"] is False, "input node publishes TF")
    need(all(p[n]["use_sim_time"] is True for n in p), "use_sim_time off somewhere")
    want_penalty = variant.get("angle_variance_penalty", active_config()["slam_toolbox"][
        "ros__parameters"]["angle_variance_penalty"])
    need(p["slam_toolbox"]["angle_variance_penalty"] == want_penalty,
         f"slam penalty {p['slam_toolbox']['angle_variance_penalty']}, wanted {want_penalty}")
    if fuses:
        # Probes older than this key: compare with the file those runs layered on.
        out_topic = p["localization_input_node"].get("imu_output_topic", active_config()[
            "localization_input_node"]["ros__parameters"]["imu_output_topic"])
        need(p["ekf_filter_node"]["imu0"] == out_topic,
             "the EKF does not read what the input node publishes")
    if "injection" in variant:
        for msg in injection_faults(out, variant["injection"]):
            need(False, msg)
    if "imu_yaw_rate_variance" in variant:
        need(p["localization_input_node"]["imu_yaw_rate_variance"] == variant["imu_yaw_rate_variance"],
             "imu_yaw_rate_variance override not in force")
    if "q_vyaw" in variant:
        need(p["ekf_filter_node"]["process_noise_covariance"][11] == variant["q_vyaw"],
             "q_vyaw override not in force")
    res = {"ok": not fails, "fails": fails, "input_duration_s": dur, "output_duration_s": dur_out,
           "counts": cout, "input_counts": cin, "slam_stall_s": stall, "ekf_freeze_s": freeze,
           "tf": {k: {**{a: b for a, b in v.items() if a not in ("changes", "yaw_changes")},
                      "n_changes": len(v["changes"])} for k, v in tf.items()}}
    (run_dir / "check.json").write_text(json.dumps(res, indent=1))
    if fails:
        raise ReplayError(f"{run_dir.name} FAILED: " + "; ".join(fails))
    return res


def parse_ekf_debug(lines):
    """robot_localization debug text -> one row per correction of the yaw rate.

    Yields (time_s, sensor, innovation, P_vyaw_before, R_vyaw, measurement). Whatever
    the update vector, in two_d_mode the measured subset carries dummy rows (z, roll,
    pitch ...), so the vyaw row is located from the printed state-to-measurement matrix.
    P is the PREDICTED covariance: "Current estimate error covariance" is printed
    before the prediction and is the previous posterior (reads R, not Q*dt). A
    measurement with no prediction of its own (stamp not newer) yields no row.
    """
    sensor, t, section, rows = None, None, None, []
    state = {}

    def finish():
        nonlocal rows, section
        if section == "H":
            for i, r in enumerate(rows):
                if len(r) >= 15 and r[11] == 1.0:
                    state["i"] = i
        elif section == "P" and len(rows) >= 12:
            state["P"] = rows[11][11]
        elif section == "R":
            state["R_rows"] = rows
        elif section == "innov" and rows:
            state["innov"] = rows[0]
        elif section == "z" and rows:
            state["z"] = rows[0]
        rows, section = [], None

    titles = {"State-to-measurement subset is:": "H",
              "Predicted estimate error covariance is:": "P",
              "Measurement covariance subset is:": "R",
              "Innovation is:": "innov", "Measurement subset is:": "z"}
    for line in lines:
        s = line.strip()
        if section is not None:
            if s:
                body = s.strip("[]").split()
                try:
                    rows.append([float(v) for v in body])
                except ValueError:
                    finish()
                    section = None
                if section is not None and s.endswith("]"):
                    finish()
                continue
            finish()
            continue
        if s in titles:
            section = titles[s]
            rows = []
        elif s.startswith("------ FilterBase::processMeasurement ("):
            sensor = s.split("(")[1].split(")")[0]
            state, t = {}, None
        elif s.startswith("Measurement time is ") and t is None:
            t = int(s.split()[3].rstrip(",")) * 1e-9
        elif s.startswith("------ /FilterBase::processMeasurement") or s.startswith(
                "---------------------- /Ekf::correct"):
            if {"i", "P", "innov", "R_rows", "z"} <= state.keys():
                i = state["i"]
                yield (t, sensor, state["innov"][i], state["P"], state["R_rows"][i][i], state["z"][i])
                state = {}


def distill_ekf_debug(run_dir):
    """Debug text -> innovations.npz; the text (about 2 MB/s of bag) is then deleted."""
    import numpy as np
    run_dir = Path(run_dir)
    txt = run_dir / "ekf_debug.txt"
    if not txt.exists():
        return
    with open(txt, errors="replace") as f:
        rows = list(parse_ekf_debug(f))
    if len(rows) < 100:
        raise ReplayError(f"{txt}: only {len(rows)} corrections parsed")
    np.savez_compressed(run_dir / "innovations.npz",
                        t=np.array([r[0] for r in rows]),
                        imu=np.array([r[1].startswith("imu") for r in rows]),
                        innovation=np.array([r[2] for r in rows]),
                        p=np.array([r[3] for r in rows]),
                        r=np.array([r[4] for r in rows]),
                        z=np.array([r[5] for r in rows]))
    txt.unlink()


def analyse_run(run_dir, env):
    """yaw_compare on the run's output, as an auditor would run it."""
    run_dir = Path(run_dir)
    distill_ekf_debug(run_dir)
    if (run_dir / "yc.json").exists():
        return
    with open(run_dir / "yc.txt", "w") as f:
        subprocess.run([sys.executable, "-m", "gripperx_localization.yaw_compare", str(run_dir / "out"),
                        "--json", str(run_dir / "yc.json"), "--csv", str(run_dir / "yc.csv")],
                       stdout=f, stderr=subprocess.STDOUT, env=env, check=True)


def plan_variant(plan, input_key, variant_key) -> dict:
    """The variant, with the time a dropped-topic input goes silent."""
    variant = dict(plan["variants"][variant_key])
    spec = plan["inputs"][input_key]
    prep = spec.get("prepare", {})
    if variant.get("expect_silent_imu"):
        variant["silent_after"] = prep.get("drop_after")
    if "gyro_inject" in prep:
        variant["injection"] = {**prep["gyro_inject"],
                                "clean_bag": str(Path(spec["bag"]).expanduser()),
                                "t0_source": bag_span(Path(spec["bag"]).expanduser())[0]}
    return variant


def recheck(args):
    """check_run again over finished runs, e.g. after the checks were tightened.
    Overwrites each run's check.json, including with a failure."""
    plan = load_plan(args.plan)
    root = Path(args.out).expanduser()
    bad = 0
    for d in sorted((root / "runs").iterdir()):
        inp, vkey, _ = d.name.split("__")
        if inp not in plan["inputs"] or vkey not in plan["variants"]:
            print(d.name, "not in this plan: skipped", flush=True)
            continue
        variant = plan_variant(plan, inp, vkey)
        try:
            res = check_run(d, root / "inputs" / inp, variant)
            print(d.name, "ok", f"stall {res['slam_stall_s']:.1f} s",
                  f"ekf gap {res['tf']['odom->base_footprint']['max_gap']:.2f} s",
                  f"freeze {res['ekf_freeze_s']:.2f} s",
                  f"slam updates {res['tf']['map->odom']['n_changes']}", flush=True)
        except ReplayError as exc:
            bad += 1
            print(exc, flush=True)
    return 1 if bad else 0


def matrix(args, env):
    plan = load_plan(args.plan)
    out_root = Path(args.out).expanduser()
    done, failed = [], []
    for stage in args.stage:
        for item in plan["stages"][stage]:
            bag = prepared_input(out_root, plan, item["input"])
            variant = plan_variant(plan, item["input"], item["variant"])
            for k in range(item.get("repeat", 1)):
                name = f"{item['input']}__{item['variant']}__{k + 1}"
                if (out_root / "runs" / name / "check.json").exists():
                    continue
                print(time.strftime("%H:%M:%S"), "run", name, flush=True)
                try:
                    run_dir = run_once(out_root, name, bag, variant, env)
                    analyse_run(run_dir, env)
                    done.append(name)
                except (ReplayError, subprocess.CalledProcessError) as exc:
                    print("FAILED:", exc, flush=True)
                    failed.append(name)
                    if not args.keep_going:
                        raise
    print(f"done {len(done)}, failed {len(failed)}: {failed}", flush=True)
    return 1 if failed else 0


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("matrix", help="run plan stages (real time, one after another)")
    m.add_argument("--stage", action="append", required=True)
    m.add_argument("--plan", default=None)
    m.add_argument("--out", default=DEFAULT_OUT)
    m.add_argument("--keep-going", action="store_true")
    s = sub.add_parser("single", help="one run of one plan variant, for a smoke test")
    s.add_argument("--input", required=True)
    s.add_argument("--variant", required=True)
    s.add_argument("--name", required=True)
    s.add_argument("--plan", default=None)
    s.add_argument("--out", default=DEFAULT_OUT)
    c = sub.add_parser("recheck", help="repeat check_run over finished runs")
    c.add_argument("--plan", default=None)
    c.add_argument("--out", default=DEFAULT_OUT)
    p = sub.add_parser("probe", help=argparse.SUPPRESS)
    p.add_argument("--out", required=True)
    return ap


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["report"]:
        from gripperx_localization.imu_replay_report import main as report_main
        isolated_env()
        return report_main(argv[1:])
    if argv[:1] == ["injection-report"]:
        from gripperx_localization.imu_replay_injection import main as injection_main
        isolated_env()
        return injection_main(argv[1:])
    args = build_parser().parse_args(argv)
    if args.cmd == "probe":
        if os.environ.get("ROS_DOMAIN_ID") != str(REPLAY_DOMAIN_ID):
            raise SystemExit("probe runs only inside the replay domain")
        return probe_main(args.out)
    try:
        env = isolated_env()
        if args.cmd == "matrix":
            return matrix(args, env)
        if args.cmd == "recheck":
            return recheck(args)
        plan = load_plan(args.plan)
        out_root = Path(args.out).expanduser()
        bag = prepared_input(out_root, plan, args.input)
        run_dir = run_once(out_root, args.name, bag, plan_variant(plan, args.input, args.variant), env)
        analyse_run(run_dir, env)
        print("ok", run_dir)
        return 0
    except (ReplayError, subprocess.CalledProcessError) as exc:
        print("REPLAY FAILED:", exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
