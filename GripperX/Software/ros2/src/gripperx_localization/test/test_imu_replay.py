"""Pure and bag-level checks of the 2.5 replay tool. The replay itself needs real time."""
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import pytest

from gripperx_localization import imu_replay as ir
from gripperx_localization.imu_replay_forensics import match_changes, match_full_circle
from gripperx_localization.imu_replay_report import innovation_summary, rate_lag, turn_sum

WRAPPER = Path(__file__).resolve().parents[3] / "scripts" / "gripperx_imu_replay.sh"


def test_env_is_forced_not_inherited():
    env = ir.isolated_env({"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0",
                           "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"})
    assert env["ROS_DOMAIN_ID"] == "87"
    assert env["ROS_LOCALHOST_ONLY"] == "1"
    assert env["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"


def test_env_refuses_robot_domain():
    with pytest.raises(ir.ReplayError):
        ir.isolated_env({"ROS_DOMAIN_ID": "20"})
    with pytest.raises(ir.ReplayError):
        ir.isolated_env({}, domain=20)


def test_wrapper_refuses_robot_domain():
    r = subprocess.run([str(WRAPPER), "report"], env={"ROS_DOMAIN_ID": "20", "PATH": "/usr/bin:/bin"},
                       capture_output=True, text=True)
    assert r.returncode == 3 and "refused" in r.stderr


def test_baseline_override_disables_imu0_only():
    o = ir.build_override({"imu0": False, "wheel_odom_yaw_rate_variance": 0.08})
    ekf = o["ekf_filter_node"]["ros__parameters"]
    assert ekf["imu0"] == "" and ekf["imu0_config"] == [False] * 15
    assert "process_noise_covariance" not in ekf
    assert o["localization_input_node"]["ros__parameters"] == {"wheel_odom_yaw_rate_variance": 0.08}


def test_merged_variant_overrides_nothing_that_matters():
    o = ir.build_override({})
    assert set(o) == {"localization_input_node"}
    assert o["localization_input_node"]["ros__parameters"] == {"use_sim_time": True}


def test_q_override_touches_the_named_index_only():
    base = [float(i) for i in range(15)]
    q = ir.build_override({"q_vyaw": 0.02}, base)["ekf_filter_node"]["ros__parameters"][
        "process_noise_covariance"]
    assert q[11] == 0.02 and q[:11] == base[:11] and q[12:] == base[12:]
    assert base[11] == 11.0


def test_plan_baseline_is_true_baseline():
    import yaml
    plan = yaml.safe_load((Path(__file__).parents[1] / "config" / "imu_replay_plan.yaml").read_text())
    assert plan["variants"]["B_baseline"]["imu0"] is False
    assert plan["variants"]["B_baseline"]["wheel_odom_yaw_rate_variance"] == 0.08
    assert plan["variants"]["A_merged"] == {}


def test_tf_filter():
    assert not ir.tf_keep("odom", "base_footprint")
    assert not ir.tf_keep("map", "odom")
    assert ir.tf_keep("chassis_link", "back_left_steer")


DEBUG = """Predicted estimate error covariance is:
[{p}]

------ FilterBase::processMeasurement (imu0_twist) ------
Measurement time is 2000000000, last measurement time is 1980000000, delta is 20000000
Current estimate error covariance is:
[{cur}]

Predicted estimate error covariance is:
[{p}]

Measurement subset is:
[0 0.5 0 ]

Measurement covariance subset is:
[1 0 0
 0 3.8e-06 0
 0 0 1 ]

State-to-measurement subset is:
[{h}]

Innovation is:
[0 0.25 0 ]

---------------------- /Ekf::correct ----------------------
"""


def _matrix(rows):
    return "\n ".join(" ".join(str(v) for v in r) for r in rows)


def test_parse_debug_takes_predicted_covariance_and_vyaw_row():
    def cov(v):
        m = [[0.0] * 15 for _ in range(15)]
        m[11][11] = v
        return _matrix(m)
    h = [[0.0] * 15 for _ in range(3)]
    h[0][2] = 1.0
    h[1][11] = 1.0
    h[2][14] = 1.0
    text = DEBUG.format(p=cov(0.01), cur=cov(3.8e-6), h=_matrix(h))
    rows = list(ir.parse_ekf_debug(text.splitlines()))
    assert len(rows) == 1
    t, sensor, inn, p, r, z = rows[0]
    assert t == pytest.approx(2.0)
    assert sensor == "imu0_twist"
    assert inn == 0.25 and p == 0.01 and r == 3.8e-6 and z == 0.5


def test_turn_sum_refuses_a_segment_without_reference():
    def seg(i, ref):
        return {"index": i, "reference": ref, "sources": {"ekf": {"error_deg": 1.0}}}
    res = {"segments": [seg(0, "scanmatch"), seg(1, "slam")]}
    assert turn_sum(res, [0], "ekf") == 1.0
    assert turn_sum(res, [0, 1], "ekf") is None


def _write_bag(path):
    import rosbag2_py
    from geometry_msgs.msg import Twist, TransformStamped
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for i, (name, typ) in enumerate((("/tf", "tf2_msgs/msg/TFMessage"),
                                     ("/tf_static", "tf2_msgs/msg/TFMessage"),
                                     ("/cmd_vel", "geometry_msgs/msg/Twist"),
                                     ("/odometry/filtered", "geometry_msgs/msg/Twist"))):
        w.create_topic(rosbag2_py.TopicMetadata(i, name, typ, "cdr"))

    def tr(parent, child):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        return t

    w.write("/tf_static", serialize_message(TFMessage(transforms=[tr("a", "b")])), 1_000_000_000)
    for k in range(5):
        ns = 1_000_000_000 + k * 400_000_000
        w.write("/tf", serialize_message(TFMessage(transforms=[tr("odom", "base_footprint"),
                                                               tr("chassis_link", "wheel")])), ns)
        w.write("/tf", serialize_message(TFMessage(transforms=[tr("map", "odom")])), ns + 1)
        w.write("/cmd_vel", serialize_message(Twist()), ns + 2)
        w.write("/odometry/filtered", serialize_message(Twist()), ns + 3)
    del w


def test_prepare_strips_old_stack_and_repeats_static(tmp_path):
    src, dst = tmp_path / "src", tmp_path / "dst"
    _write_bag(src)
    info = ir.prepare(src, dst)
    assert info["removed_tf"] == 10
    assert info["counts"]["/cmd_vel"] == 5
    ir.check_prepared(dst)
    counts = ir._meta_counts(dst)
    assert "/odometry/filtered" not in counts
    assert counts["/tf_static"] >= 2
    assert counts["/tf"] == 5


def test_check_prepared_catches_a_survivor(tmp_path):
    src = tmp_path / "src"
    _write_bag(src)
    with pytest.raises(ir.ReplayError):
        ir.check_prepared(src)


def test_prepare_never_overwrites(tmp_path):
    src = tmp_path / "src"
    _write_bag(src)
    with pytest.raises(ir.ReplayError):
        ir.prepare(src, src)


# ------------------------------------------------------------------ prepare options

def _sensor_bag(path, seconds=8, imu_hz=10):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_msgs.msg import TFMessage

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(0, "/tf_static", "tf2_msgs/msg/TFMessage", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(1, "/scan", "sensor_msgs/msg/LaserScan", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(2, "/imu/bno085/data", "sensor_msgs/msg/Imu", "cdr"))
    t0 = 1_000_000_000
    w.write("/tf_static", serialize_message(TFMessage()), t0)
    for k in range(seconds * 10):
        ns = t0 + k * 100_000_000
        m = LaserScan(ranges=[1.0] * (453 + k % 2))
        m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
        w.write("/scan", serialize_message(m), ns)
    for k in range(seconds * imu_hz):
        ns = t0 + k * (10**9 // imu_hz)
        w.write("/imu/bno085/data", serialize_message(Imu()), ns)
    del w


def _scans(uri):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    r, out = ir._reader(uri, ["/scan"]), []
    while r.has_next():
        _, raw, t = r.read_next()
        m = deserialize_message(raw, LaserScan)
        out.append((t * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, len(m.ranges)))
    return out


def test_scan_lead_size_keeps_one_size_in_the_lead_only(tmp_path):
    src, dst = tmp_path / "s", tmp_path / "d"
    _sensor_bag(src)
    ir.prepare(src, dst, scan_lead_size=454, scan_lead_sec=3.0)
    rows = _scans(dst)
    t0 = rows[0][0] - 0.1          # the first kept scan is the second recorded
    lead = [n for t, _, n in rows if t - t0 < 3.0]
    after = [n for t, _, n in rows if t - t0 >= 3.0]
    assert set(lead) == {454} and set(after) == {453, 454}


def test_scan_stamp_shift_moves_header_not_receive_time(tmp_path):
    src, dst = tmp_path / "s", tmp_path / "d"
    _sensor_bag(src)
    ir.prepare(src, dst, scan_stamp_shift=-0.05)
    for recv, stamp, _ in _scans(dst):
        assert stamp - recv == pytest.approx(-0.05, abs=1e-6)


def test_drop_topic_silences_only_that_topic_from_that_time(tmp_path):
    src, dst = tmp_path / "s", tmp_path / "d"
    _sensor_bag(src)
    info = ir.prepare(src, dst, drop_topic="/imu/bno085/data", drop_after=3.0)
    assert info["counts"]["/imu/bno085/data"] == 30
    assert info["counts"]["/scan"] == 80


# ---------------------------------------------------------------------- check_run

def _good_run(root, duration=30, **broken):
    """A run directory holding what a healthy replay leaves behind, or a broken one."""
    import rosbag2_py
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from tf2_msgs.msg import TFMessage

    def stamp(h, ns):
        h.stamp.sec, h.stamp.nanosec = ns // 10**9, ns % 10**9

    topics = (("/scan", "sensor_msgs/msg/LaserScan"), ("/imu/bno085/data", "sensor_msgs/msg/Imu"),
              ("/imu/data/filtered", "sensor_msgs/msg/Imu"), ("/wheel/odom", "nav_msgs/msg/Odometry"),
              ("/odometry/filtered", "nav_msgs/msg/Odometry"), ("/tf", "tf2_msgs/msg/TFMessage"),
              ("/tf_static", "tf2_msgs/msg/TFMessage"),
              ("/joint_states", "sensor_msgs/msg/JointState"))

    def bag(uri, names):
        w = rosbag2_py.SequentialWriter()
        w.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
               rosbag2_py.ConverterOptions("cdr", "cdr"))
        for i, (n, t) in enumerate(x for x in topics if x[0] in names):
            w.create_topic(rosbag2_py.TopicMetadata(i, n, t, "cdr"))
        return w

    def tf(parent, child, ns, value):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        stamp(t.header, ns)
        t.transform.rotation.z, t.transform.rotation.w = value, 1.0
        return TFMessage(transforms=[t])

    t0 = 1_000_000_000_000
    inp = root / "in"
    i = bag(inp, {"/scan", "/imu/bno085/data", "/tf_static", "/joint_states"})
    run = root / "run"
    run.mkdir()
    o = bag(run / "out", {n for n, _ in topics})
    o.write("/tf_static", serialize_message(TFMessage()), t0)
    silent_ns = broken.get("imu_stops_at")
    for k in range(duration * 50):
        ns = t0 + k * 20_000_000
        odom = Odometry()
        stamp(odom.header, ns)
        odom.twist.twist.angular.z = 1.0
        o.write("/wheel/odom", serialize_message(odom), ns)
        if broken.get("foreign_odom") and k % 2 == 0:
            stray = Odometry()                 # another node on the same topic, its own clock
            stamp(stray.header, ns - 5_000_000_000)
            o.write("/wheel/odom", serialize_message(stray), ns)
        i.write("/joint_states", serialize_message(JointState()), ns)
        frozen_ekf = broken.get("ekf_gap") and 10 * 50 <= k < 15 * 50
        if not frozen_ekf:
            o.write("/odometry/filtered", serialize_message(odom), ns)
            yaw = 0.25 if broken.get("ekf_yaw_frozen") and k > 12 * 50 else k * 1e-4
            o.write("/tf", serialize_message(tf("odom", "base_footprint", ns, yaw)), ns)
        imu = Imu()
        stamp(imu.header, ns)
        if silent_ns is None or k * 0.02 < silent_ns:
            i.write("/imu/bno085/data", serialize_message(imu), ns)
            o.write("/imu/data/filtered", serialize_message(imu), ns)
        if k % 5 == 0:
            scan = LaserScan()
            stamp(scan.header, ns)
            i.write("/scan", serialize_message(scan), ns)
            if not (broken.get("scans_lost") and k > 25 * 50):
                o.write("/scan", serialize_message(scan), ns)
            slam_alive = not (broken.get("slam_stalls") and k > 8 * 50)
            every = 250 if broken.get("slam_sparse") else 25    # 12 or 120 updates per minute
            value = (k // every) * 1e-3 if slam_alive else 0.5
            o.write("/tf", serialize_message(tf("map", "odom", ns + 500_000_000, value)), ns)
    i.write("/tf_static", serialize_message(TFMessage()), t0)
    del i, o
    pub = {"/tf": ["ekf_filter_node", "rosbag2_player", "slam_toolbox", "slam_toolbox"],
           "/wheel/odom": ["localization_input_node"], "/odometry/filtered": ["ekf_filter_node"],
           "/imu/data/filtered": ["localization_input_node"]}
    sub = {"/tf": [], "/wheel/odom": ["ekf_filter_node"], "/odometry/filtered": [],
           "/imu/data/filtered": ["ekf_filter_node"]}
    (run / "probe.json").write_text(json.dumps({
        "graph": {k: {"pub": pub[k], "sub": sub[k]} for k in pub},
        "params": {"ekf_filter_node": {"use_sim_time": True, "imu0": "/imu/data/filtered"},
                   "localization_input_node": {"publish_tf": False, "use_sim_time": True,
                                               "imu_output_topic": "/imu/data/filtered"},
                   "slam_toolbox": {"angle_variance_penalty": 2.0, "use_sim_time": True}}}))
    return run, inp


def test_check_run_passes_a_healthy_run(tmp_path):
    run, inp = _good_run(tmp_path)
    res = ir.check_run(run, inp, {})
    assert res["ok"] and json.loads((run / "check.json").read_text())["ok"]


@pytest.mark.parametrize("broken, text", [
    ({"ekf_gap": True}, "EKF silent"),
    ({"slam_stalls": True}, "slam idle"),
    ({"scans_lost": True}, "/scan"),
    ({"foreign_odom": True}, "/wheel/odom"),
    ({"ekf_yaw_frozen": True}, "EKF yaw frozen"),
    ({"slam_sparse": True}, "slam made"),
])
def test_check_run_fails_loudly(tmp_path, broken, text):
    run, inp = _good_run(tmp_path, **broken)
    with pytest.raises(ir.ReplayError, match=text):
        ir.check_run(run, inp, {})


def test_check_run_catches_a_mid_run_ekf_stop_a_whole_run_check_would_pass(tmp_path):
    run, inp = _good_run(tmp_path, ekf_gap=True)
    with pytest.raises(ir.ReplayError):
        ir.check_run(run, inp, {})
    assert json.loads((run / "check.json").read_text())["tf"]["odom->base_footprint"]["n_changes"] > 0


def test_check_run_refuses_a_silent_run_whose_imu_kept_talking(tmp_path):
    run, inp = _good_run(tmp_path)            # IMU runs to the end
    with pytest.raises(ir.ReplayError, match="not silent"):
        ir.check_run(run, inp, {"expect_silent_imu": True, "silent_after": 10.0})


def test_check_run_accepts_a_real_silent_run(tmp_path):
    run, inp = _good_run(tmp_path, imu_stops_at=20.0)
    assert ir.check_run(run, inp, {"expect_silent_imu": True, "silent_after": 20.0})["ok"]


def test_check_run_refuses_a_silent_run_that_went_silent_too_early(tmp_path):
    run, inp = _good_run(tmp_path, imu_stops_at=2.0)
    with pytest.raises(ir.ReplayError, match="samples before going silent"):
        ir.check_run(run, inp, {"expect_silent_imu": True, "silent_after": 20.0})


def test_matrix_keep_going_survives_a_failed_analysis(tmp_path, monkeypatch):
    plan = {"inputs": {"x": {"bag": "/nowhere"}}, "variants": {"v": {}},
            "stages": {"s": [{"input": "x", "variant": "v"}]}}
    monkeypatch.setattr(ir, "load_plan", lambda _: plan)
    monkeypatch.setattr(ir, "prepared_bag", lambda *a, **k: "bag")
    monkeypatch.setattr(ir, "run_once", lambda *a, **k: "dir")

    def boom(*a):
        raise subprocess.CalledProcessError(1, "yaw_compare")
    monkeypatch.setattr(ir, "analyse_run", boom)
    args = type("A", (), {"plan": None, "out": str(tmp_path), "stage": ["s"], "keep_going": True})
    assert ir.matrix(args, {}) == 1


def test_check_run_refuses_a_silent_run_without_its_time(tmp_path):
    run, inp = _good_run(tmp_path)
    with pytest.raises(ir.ReplayError, match="silent_after"):
        ir.check_run(run, inp, {"expect_silent_imu": True})


def test_graph_complete_waits_for_slam_on_tf():
    g = {"/tf": {"pub": ["ekf_filter_node", "rosbag2_player"], "sub": []},
         "/wheel/odom": {"pub": ["localization_input_node"], "sub": ["ekf_filter_node"]},
         "/odometry/filtered": {"pub": ["ekf_filter_node"], "sub": []},
         "/imu/data/filtered": {"pub": ["localization_input_node"], "sub": []}}
    assert not ir.graph_complete(g)
    g["/tf"]["pub"].append("slam_toolbox")
    assert ir.graph_complete(g)
    g["/tf"]["pub"].append("_NODE_NAME_UNKNOWN_")
    assert not ir.graph_complete(g)


def test_plan_variant_carries_the_drop_time():
    plan = {"inputs": {"x": {"prepare": {"drop_after": 12.0}}},
            "variants": {"s": {"expect_silent_imu": True}, "n": {}}}
    assert ir.plan_variant(plan, "x", "s")["silent_after"] == 12.0
    assert "silent_after" not in ir.plan_variant(plan, "x", "n")


# --------------------------------------------------------------- analysis functions

def test_innovation_summary_counts_gates_and_spin_edges(tmp_path):
    n = 1000
    t = np.arange(n) * 0.02
    z = np.where((t > 5) & (t < 15), 1.0, 0.0)
    inn = np.zeros(n)
    inn[300] = 1.6                      # 16 sigma, far from an edge
    inn[250] = 0.6                      # 6 sigma, at the spin start
    path = tmp_path / "i.npz"
    np.savez(path, t=t, imu=np.ones(n, bool), innovation=inn, p=np.full(n, 0.01),
             r=np.zeros(n), z=z)
    # npz needs odometry rows too; the summary reads both populations.
    d = dict(np.load(path))
    d = {k: np.concatenate([v, v[:5]]) for k, v in d.items()}
    d["imu"][-5:] = False
    np.savez(path, **d)
    res = innovation_summary(path)
    assert res["n"] == n and res["n_edges"] == 2
    assert res["over_gate"] == {"5": 2, "10": 1, "15": 1, "20": 0}
    assert res["over_gate_at_edges"]["5"] == 1 and res["over_gate_at_edges"]["10"] == 0
    assert res["gate_width_rad_s"]["15"] == pytest.approx(1.5)
    assert res["largest"][0]["nu"] == pytest.approx(16.0)


def test_rate_lag_recovers_a_known_delay():
    g = np.arange(0.0, 20.0, 0.02)
    x = np.sin(g) + np.sin(3.1 * g)
    assert rate_lag(g, x, np.interp(g - 0.06, g, x)) == pytest.approx(0.06, abs=0.004)


def test_match_changes_finds_the_stamp_offset():
    # Jitter is what separates 0.5 s from 0.4 s: a perfectly periodic scan cannot.
    scans = np.cumsum(0.1 + np.random.default_rng(3).normal(0, 0.004, 1000))
    off, resid, j = match_changes(scans[::7] + 0.5, scans)
    assert off == pytest.approx(0.5, abs=0.011) and resid < 0.01
    assert list(j[:3]) == [0, 7, 14]


def test_full_circle_match_recovers_a_rotation_without_a_guess():
    rng = np.random.default_rng(1)
    a = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    r = 2.0 + 0.8 * np.sin(3 * a) + 0.4 * np.cos(7 * a)
    room = np.c_[r * np.cos(a), r * np.sin(a)] + rng.normal(0, 0.002, (180, 2))
    yaw = math.radians(137.0)
    c, s = math.cos(yaw), math.sin(yaw)
    after = room @ np.array([[c, -s], [s, c]])    # the room as seen after turning +137 deg
    score, got = match_full_circle(room, after, np.zeros(2))
    assert got == pytest.approx(137.0, abs=0.5) and score < 0.02
