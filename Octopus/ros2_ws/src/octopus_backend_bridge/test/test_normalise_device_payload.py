"""The GripperX dialect translation in device_status_backend_bridge_node.

These cases are the ones that would look wrong on the mission map, so they are
asserted rather than eyeballed. No ROS graph is needed - the functions under
test are pure.
"""

from octopus_backend_bridge.device_status_backend_bridge_node import (
    looks_like_gripperx_dialect,
    normalise_device_payload,
)


def gripperx_payload(pose_status="available", latlon_status="available",
                     lat=48.2513611, lon=11.6359722, nav_state="idle"):
    """The shape octopus_protocol.build_device_status actually emits."""
    return {
        "device_id": "gripperx",
        "stamp": 1787234926.3,
        "pose": {
            "status": pose_status,
            "reason": "",
            "lat": lat,
            "lon": lon,
            "latlon_status": latlon_status,
            "latlon_reason": "",
            "x": 1.2, "y": 0.4, "yaw_deg": 35.0, "speed_mps": 0.0,
        },
        "nav_state": nav_state,
        "active_goal_id": None,
        "armed": False,
        "link_ok": True,
        "link": {"last_message_age_sec": 0.7, "reconnects": 0},
        "battery": {"status": "unavailable", "reason": "NO_SENSOR_INSTALLED",
                    "percent": None},
    }


def test_healthy_robot_becomes_ok():
    out = normalise_device_payload(gripperx_payload())
    assert out["pose"]["status"] == "ok"
    assert out["robot_id"] == "gripperx"
    assert out["timestamp"] == 1787234926.3
    assert out["nav"]["status"] == "idle"
    assert out["link"]["connected"] is True


def test_missing_datum_is_no_datum():
    out = normalise_device_payload(
        gripperx_payload(latlon_status="unavailable", lat=None, lon=None))
    assert out["pose"]["status"] == "no_datum"


def test_broken_pose_reports_no_pose_not_no_datum():
    """A broken TF also invalidates the lat/lon, and naming that "no_datum"
    would point the operator at a datum that is fine."""
    out = normalise_device_payload(
        gripperx_payload(pose_status="unavailable", latlon_status="unavailable",
                         lat=None, lon=None))
    assert out["pose"]["status"] == "no_pose"


def test_normalisation_is_additive():
    out = normalise_device_payload(gripperx_payload())
    assert out["device_id"] == "gripperx"       # original kept
    assert out["nav_state"] == "idle"           # original kept
    assert out["pose"]["source_status"] == "available"


def test_distance_remaining_is_null_not_invented():
    out = normalise_device_payload(gripperx_payload())
    assert out["nav"]["distance_remaining_m"] is None


def test_dashboard_dialect_passes_through_untouched():
    """Keeps this node generic: another robot already speaking the dashboard
    shape must not be rewritten."""
    native = {"robot_id": "robby", "timestamp": 1.0,
              "pose": {"status": "ok", "lat": 1.0, "lon": 2.0},
              "nav": {"status": "navigating"}}
    before = repr(native)
    out = normalise_device_payload(native)
    assert repr(out) == before
    assert not looks_like_gripperx_dialect(native)


def test_nav_state_vocabulary_survives():
    """All five GripperX nav states reach the dashboard verbatim; classifying
    them is live_data.js's job, not this node's."""
    for state in ("idle", "navigating", "picking", "cancelling", "unavailable"):
        out = normalise_device_payload(gripperx_payload(nav_state=state))
        assert out["nav"]["status"] == state
