from fastapi import FastAPI




from fastapi.staticfiles import StaticFiles
import sqlite3
from pydantic import BaseModel
from typing import List, Dict, Any
import os
from datetime import datetime

DB_PATH = os.getenv("DB_PATH", "octopusfinal.db")
app = FastAPI()


# --- CAMERA DEBUG / DETECTION INSPECTOR ROUTES ---
# In-memory only: keep the newest frame and newest detection payload.
LATEST_CAMERA_DEBUG = {
    "received_at": None,
    "image": None,
    "detections": None,
}


def _now_ts() -> float:
    return datetime.now().timestamp()


# --- CAMERA CROP CONFIG ---
# The operator sets the crop in the dashboard's "Camera & Pipeline" panel. The
# dashboard POSTs it here, the ROS camera-debug bridge polls it, and the bridge
# cuts the frame edges away BEFORE the frame is sent — so Eve only ships the part
# of the feed the operator actually wants, instead of a full frame that the
# browser then hides.
#
# Each side is a fraction of the full frame (0.0 .. 0.45), capped so the cropped
# region always still contains the camera's principal point.
CAMERA_CROP_SIDES = ("top", "right", "bottom", "left")
CAMERA_CROP_MAX_SIDE = 0.45

CAMERA_CROP: Dict[str, Any] = {
    "top": 0.0,
    "right": 0.0,
    "bottom": 0.0,
    "left": 0.0,
    # Bumped on every real change, so the bridge can log only actual switches.
    "revision": 0,
    "updated_at": None,
}


def _camera_crop_side(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if number != number:  # NaN
        return 0.0
    return max(0.0, min(number, CAMERA_CROP_MAX_SIDE))


def _camera_crop_public() -> Dict[str, Any]:
    crop = {side: CAMERA_CROP[side] for side in CAMERA_CROP_SIDES}
    crop["revision"] = CAMERA_CROP["revision"]
    crop["updated_at"] = CAMERA_CROP["updated_at"]
    crop["active"] = any(CAMERA_CROP[side] > 0.0 for side in CAMERA_CROP_SIDES)
    crop["max_side"] = CAMERA_CROP_MAX_SIDE
    return crop


@app.get("/api/camera_debug/crop")
def get_camera_debug_crop():
    return {"status": "ok", "crop": _camera_crop_public()}


@app.post("/api/camera_debug/crop")
def post_camera_debug_crop(payload: Dict[str, Any]):
    changed = False
    for side in CAMERA_CROP_SIDES:
        if side not in payload:
            continue
        value = _camera_crop_side(payload[side])
        if abs(value - CAMERA_CROP[side]) > 1e-9:
            CAMERA_CROP[side] = value
            changed = True

    if changed:
        CAMERA_CROP["revision"] += 1
        CAMERA_CROP["updated_at"] = _now_ts()

    return {"status": "ok", "changed": changed, "crop": _camera_crop_public()}


@app.post("/api/camera_debug/frame")
def post_camera_debug_frame(payload: Dict[str, Any]):
    received_at = _now_ts()
    image_format = str(payload.get("format") or "jpeg").lower()
    if image_format == "jpg":
        image_format = "jpeg"

    data_url = payload.get("data_url")
    data_base64 = payload.get("data_base64")
    if not data_url and data_base64:
        data_url = f"data:image/{image_format};base64,{data_base64}"

    # The crop the bridge actually applied to THIS frame, so the dashboard knows
    # whether the image it got is already cut (draw it as-is) or still full
    # (crop it in the browser as a fallback).
    applied_crop = payload.get("crop")
    if isinstance(applied_crop, dict):
        applied_crop = {side: _camera_crop_side(applied_crop.get(side)) for side in CAMERA_CROP_SIDES}
    else:
        applied_crop = None

    LATEST_CAMERA_DEBUG["image"] = {
        "received_at": received_at,
        "format": image_format,
        "stamp": payload.get("stamp"),
        "frame_id": payload.get("frame_id", "camera"),
        "data_url": data_url,
        "crop": applied_crop,
        "source_size": payload.get("source_size"),
        "cropped_size": payload.get("cropped_size"),
        "bytes": len(data_base64) if isinstance(data_base64, str) else None,
    }
    LATEST_CAMERA_DEBUG["received_at"] = received_at

    return {
        "status": "ok",
        "received_at": received_at,
        "has_image": bool(data_url),
        # Piggy-back the current crop config on the response, so a bridge that
        # posts often does not need a separate poll to stay current.
        "crop": _camera_crop_public(),
    }


@app.get("/api/camera_debug/frame/latest")
def get_latest_camera_debug_frame():
    return {
        "status": "ok" if LATEST_CAMERA_DEBUG["image"] else "empty",
        "image": LATEST_CAMERA_DEBUG["image"],
    }


@app.post("/api/camera_debug/detections")
def post_camera_debug_detections(payload: Dict[str, Any]):
    received_at = _now_ts()
    payload["received_at"] = payload.get("received_at", received_at)
    payload.setdefault("source_id", "detector_node")
    payload.setdefault("frame_id", "camera")
    payload.setdefault("detections", [])

    LATEST_CAMERA_DEBUG["detections"] = payload
    LATEST_CAMERA_DEBUG["received_at"] = received_at

    return {
        "status": "ok",
        "received_at": received_at,
        "detection_count": len(payload.get("detections", [])),
    }


@app.get("/api/camera_debug/detections/latest")
def get_latest_camera_debug_detections():
    return {
        "status": "ok" if LATEST_CAMERA_DEBUG["detections"] else "empty",
        "detections": _camera_debug_detections_public(),
    }


# --- CHEAT: KONFIDENZ-ZUSCHLAG ---
# Fuer Vorfuehrungen: jede gemeldete Konfidenz wird um DETECTOR_CONFIG
# ["confidence_bonus"] angehoben (negativ geht auch).
#
# Bewusst BEIM AUSLIEFERN und nicht beim Speichern: LATEST_CAMERA_DEBUG behaelt
# die echten Werte, ein Zuschlag von 0 liefert sofort wieder die Wahrheit, und
# nichts anderes im System sieht die frisierten Zahlen -- weder der Kartenlayer
# noch die Ziele fuer den Sammelroboter, die beide ueber ROS laufen. Der
# Zuschlag ist reine Anzeige.
#
# Jede angehobene Detektion behaelt ihren Rohwert in confidence_raw und traegt
# confidence_faked, damit im Zweifel nachvollziehbar bleibt, was echt war.
def _detector_confidence_bonus() -> float:
    try:
        return float(DETECTOR_CONFIG.get("confidence_bonus", 0.0))
    except (TypeError, ValueError):
        return 0.0


def _camera_debug_detections_public():
    payload = LATEST_CAMERA_DEBUG["detections"]
    bonus = _detector_confidence_bonus()

    if not payload or abs(bonus) < 1e-9:
        return payload

    items = payload.get("detections")
    if not isinstance(items, list):
        return payload

    faked = []
    for det in items:
        if not isinstance(det, dict):
            faked.append(det)
            continue
        raw = det.get("confidence")
        try:
            value = float(raw)
        except (TypeError, ValueError):
            faked.append(det)
            continue
        copy = dict(det)
        copy["confidence"] = max(0.0, min(1.0, value + bonus))
        copy["confidence_raw"] = value
        copy["confidence_faked"] = True
        faked.append(copy)

    # Flache Kopie: die gespeicherte Nutzlast darf nicht veraendert werden.
    out = dict(payload)
    out["detections"] = faked
    out["confidence_bonus"] = bonus
    return out


@app.get("/api/camera_debug/latest")
def get_latest_camera_debug():
    return {
        "status": "ok" if (LATEST_CAMERA_DEBUG["image"] or LATEST_CAMERA_DEBUG["detections"]) else "empty",
        "received_at": LATEST_CAMERA_DEBUG["received_at"],
        "image": LATEST_CAMERA_DEBUG["image"],
        "detections": _camera_debug_detections_public(),
        # So the dashboard can tell whether the backend already knows its crop —
        # after a backend restart the dashboard re-POSTs it from localStorage.
        "crop": _camera_crop_public(),
    }
# --- END CAMERA DEBUG / DETECTION INSPECTOR ROUTES ---


# --- OCTOPUS EVE CAMERA ROUTES ---
import subprocess
from datetime import datetime


EVE_SSH_TARGET = os.environ.get("OCTOPUS_EVE_SSH_TARGET", "eve-pi")

# Die Kamera-Skripte liegen im Pi-Repo (Branch eve_ros_development), neben dem
# camera_pkg, das sie starten. Keine Leerzeichen im Pfad: er geht unquoted in
# die Remote-Shell, damit ~ dort expandiert wird.
EVE_SCRIPT_DIR = os.environ.get(
    "OCTOPUS_EVE_SCRIPT_DIR", "~/PlastiX/eve/Software/scripts"
)

# Dahin schreibt octopus_start_camera.sh das Log des camera_node.
EVE_CAMERA_LOG = "/tmp/octopus_camera_node.log"


def _octopus_eve_script(script_name: str) -> str:
    """Pfad eines Kamera-Skripts auf der Pi."""
    return f"{EVE_SCRIPT_DIR}/{script_name}"


def _octopus_run_eve_command(command: str, timeout: int = 10):
    ssh_command = [
        "ssh",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=4",
        EVE_SSH_TARGET,
        command,
    ]

    try:
        result = subprocess.run(
            ssh_command,
            capture_output=True,
            text=True,
            timeout=timeout,
        )

        return {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
            "timestamp": datetime.now().isoformat(),
        }

    except subprocess.TimeoutExpired:
        return {
            "ok": False,
            "returncode": -1,
            "stdout": "",
            "stderr": "Command timed out",
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as exc:
        return {
            "ok": False,
            "returncode": -1,
            "stdout": "",
            "stderr": str(exc),
            "timestamp": datetime.now().isoformat(),
        }


@app.get("/api/eve/status")
def octopus_eve_status():
    result = _octopus_run_eve_command(
        _octopus_eve_script("octopus_camera_status.sh"), timeout=8
    )

    status = "offline"
    if result["ok"]:
        if "camera_running" in result["stdout"]:
            status = "camera_running"
        elif "camera_not_running" in result["stdout"]:
            status = "online_camera_stopped"
        else:
            status = "online_unknown"

    return {
        "status": status,
        "ssh": result,
    }


@app.post("/api/eve/start_camera")
def octopus_eve_start_camera():
    result = _octopus_run_eve_command(
        _octopus_eve_script("octopus_start_camera.sh"), timeout=15
    )

    status = (
        "camera_started"
        if result["ok"] and "camera_started" in result["stdout"]
        else "camera_failed"
    )

    return {
        "status": status,
        "ssh": result,
    }


@app.post("/api/eve/stop_camera")
def octopus_eve_stop_camera():
    result = _octopus_run_eve_command(
        _octopus_eve_script("octopus_stop_camera.sh"), timeout=10
    )

    status = (
        "camera_stopped"
        if result["ok"] and (
            "camera_stopped" in result["stdout"]
            or "camera_not_running" in result["stdout"]
        )
        else "camera_stop_failed"
    )

    return {
        "status": status,
        "ssh": result,
    }


@app.get("/api/eve/camera_log")
def octopus_eve_camera_log():
    result = _octopus_run_eve_command(f"tail -80 {EVE_CAMERA_LOG}", timeout=8)

    return {
        "status": "ok" if result["ok"] else "failed",
        "log": result["stdout"],
        "ssh": result,
    }
# --- END OCTOPUS EVE CAMERA ROUTES ---

# --- OCTOPUS EVE PX4 BRIDGE ROUTES ---
# Der MicroXRCEAgent auf der Pi, Terminal 1 aus docs/SETUP.md. Braucht kein
# sudo: /dev/ttyAMA0 gehoert root:dialout und der Benutzer eve ist darin -- nur
# deshalb laesst er sich ueberhaupt von hier aus starten, denn der SSH-Aufruf
# ist nicht interaktiv und wuerde an einer Passwortabfrage haengen bleiben.
EVE_PX4_BRIDGE_LOG = "/tmp/octopus_px4_bridge.log"


@app.get("/api/eve/px4_bridge/status")
def octopus_eve_px4_bridge_status():
    result = _octopus_run_eve_command(
        _octopus_eve_script("octopus_px4_bridge_status.sh"), timeout=8
    )

    status = "offline"
    if result["ok"]:
        if "px4_bridge_running" in result["stdout"]:
            status = "px4_bridge_running"
        elif "px4_bridge_not_running" in result["stdout"]:
            status = "online_px4_bridge_stopped"
        else:
            status = "online_unknown"

    return {
        "status": status,
        "ssh": result,
    }


@app.post("/api/eve/px4_bridge/start")
def octopus_eve_px4_bridge_start():
    result = _octopus_run_eve_command(
        _octopus_eve_script("octopus_start_px4_bridge.sh"), timeout=20
    )

    status = (
        "px4_bridge_started"
        if result["ok"] and "px4_bridge_started" in result["stdout"]
        else "px4_bridge_failed"
    )

    # Der Agent laeuft auch ohne Pixhawk am anderen Ende -- er wartet dann nur.
    # Fuer die Demo ist das der interessantere Zustand, deshalb getrennt
    # gemeldet statt in "started" verschwiegen.
    pixhawk = None
    if "pixhawk=connected" in result["stdout"]:
        pixhawk = "connected"
    elif "pixhawk=waiting" in result["stdout"]:
        pixhawk = "waiting"

    return {
        "status": status,
        "pixhawk": pixhawk,
        "ssh": result,
    }


@app.post("/api/eve/px4_bridge/stop")
def octopus_eve_px4_bridge_stop():
    result = _octopus_run_eve_command(
        _octopus_eve_script("octopus_stop_px4_bridge.sh"), timeout=15
    )

    status = (
        "px4_bridge_stopped"
        if result["ok"] and (
            "px4_bridge_stopped" in result["stdout"]
            or "px4_bridge_not_running" in result["stdout"]
        )
        else "px4_bridge_stop_failed"
    )

    return {
        "status": status,
        "ssh": result,
    }


@app.get("/api/eve/px4_bridge/log")
def octopus_eve_px4_bridge_log():
    result = _octopus_run_eve_command(f"tail -80 {EVE_PX4_BRIDGE_LOG}", timeout=8)

    return {
        "status": "ok" if result["ok"] else "failed",
        "log": result["stdout"],
        "ssh": result,
    }
# --- END OCTOPUS EVE PX4 BRIDGE ROUTES ---



# --- OCTOPUS EVE FAKE GPS START COORDINATE ---
# Eve's placement on the mission map lives in the browser (localStorage), but the
# collector robot needs it in ROS: it is started at the same physical spot, so
# Eve's fake coordinate is the shared datum every trash goal is relative to.
# The dashboard posts it here on every placement, eve_fake_gps_bridge_node polls
# it and publishes /octopus/fake_eve_gps_start.
EVE_FAKE_GPS = None


@app.post("/api/eve/fake_gps")
def post_eve_fake_gps(payload: Dict[str, Any]):
    global EVE_FAKE_GPS

    EVE_FAKE_GPS = dict(payload)
    EVE_FAKE_GPS["backend_received_at"] = _now_ts()

    return {"status": "ok", "eve_fake_gps": EVE_FAKE_GPS}


@app.get("/api/eve/fake_gps")
def get_eve_fake_gps():
    if EVE_FAKE_GPS is None:
        return {
            "status": "empty",
            "message": "No Eve position posted by the dashboard yet.",
            "eve_fake_gps": None,
        }

    return {"status": "ok", "eve_fake_gps": EVE_FAKE_GPS}
# --- END OCTOPUS EVE FAKE GPS START COORDINATE ---


# --- DEVICE STATUS (GROUND ROBOTS) ---
# Ground robots on the GripperX link publish their own state as a JSON string on
# /octopus/devices/<id>/status. device_status_backend_bridge_node forwards each
# message here, the dashboard polls /api/devices/status once per refresh and puts
# the robot on the mission map.
#
# In-memory and newest-wins per device: this is live telemetry, not a log. A
# robot that stops publishing keeps its last payload, and the dashboard decides
# from backend_received_at whether that is still current - so a dead link shows
# as stale rather than silently disappearing.
DEVICE_STATUS: Dict[str, Dict[str, Any]] = {}

# A robot id becomes part of a URL and a dashboard label, so keep it boring.
DEVICE_ID_MAX_LEN = 64


def _clean_device_id(device_id: Any) -> str:
    cleaned = "".join(
        char for char in str(device_id or "").strip().lower()
        if char.isalnum() or char in "-_"
    )
    return cleaned[:DEVICE_ID_MAX_LEN]


@app.post("/api/devices/{device_id}/status")
def post_device_status(device_id: str, payload: Dict[str, Any]):
    clean_id = _clean_device_id(device_id)
    if not clean_id:
        return {"status": "error", "message": "Unusable device_id", "device_id": device_id}

    record = dict(payload)
    record["device_id"] = clean_id
    record["backend_received_at"] = _now_ts()
    DEVICE_STATUS[clean_id] = record

    return {"status": "ok", "device_id": clean_id, "device_status": record}


@app.get("/api/devices/status")
def get_all_device_status():
    """Every known device in one call - what the dashboard polls."""
    return {
        "status": "ok",
        "server_time": _now_ts(),
        "count": len(DEVICE_STATUS),
        "devices": DEVICE_STATUS,
    }


@app.get("/api/devices/{device_id}/status")
def get_device_status(device_id: str):
    clean_id = _clean_device_id(device_id)
    record = DEVICE_STATUS.get(clean_id)
    if record is None:
        return {
            "status": "empty",
            "message": f"No status posted for device '{clean_id}' yet.",
            "device_id": clean_id,
            "server_time": _now_ts(),
            "device_status": None,
        }

    return {
        "status": "ok",
        "device_id": clean_id,
        "server_time": _now_ts(),
        "device_status": record,
    }
# --- END DEVICE STATUS (GROUND ROBOTS) ---



# Store server start time
SERVER_START_TIME = datetime.now()

def query_db(query, args=()):
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cur = conn.cursor()
    cur.execute(query, args)
    rows = [dict(r) for r in cur.fetchall()]
    conn.close()
    return rows

@app.get("/api/locations")
def get_locations(limit: int = 100):
    return query_db("SELECT origin_id AS id, type, latitude AS lat, longitude AS lon, altitude AS alt, timestamp AS ts FROM locations ORDER BY timestamp DESC LIMIT ?", (limit,))

@app.get("/api/tasks")
def get_tasks():
    rows = query_db("SELECT id, latitude AS lat, longitude AS lon, assigned_to AS assigned, status, timestamp AS ts FROM tasks ORDER BY timestamp DESC LIMIT 100")
    # fallback sample if no table
    if not rows:
        return [
           {"id":1,"lat":48.137,"lon":11.576,"assigned":"robot_1","status":"in_progress","ts":"2025-11-13T12:00:00Z"}
        ]
    return rows

@app.get("/api/battery")
def get_battery():
    rows = query_db("SELECT device_id AS id, battery_percent AS percent, state, timestamp AS ts FROM battery ORDER BY timestamp DESC LIMIT 100")
    if not rows:
        return [
          {"id":"drone_1","percent":87,"state":"active","ts":"2025-11-13T12:05:00Z"},
          {"id":"robot_1","percent":65,"state":"active","ts":"2025-11-13T12:04:00Z"}
        ]
    return rows

@app.get("/api/stats")
def get_stats():
    # Count active robots (exclude laptop)
    robots = query_db("SELECT COUNT(DISTINCT device_id) as count FROM battery WHERE type='robot'")
    
    # Count active drones
    drones = query_db("SELECT COUNT(DISTINCT device_id) as count FROM battery WHERE type='drone'")
    
    # Count completed tasks
    collected = query_db("SELECT COUNT(*) as count FROM tasks WHERE status='completed'")
    
    # Count pending tasks
    pending = query_db("SELECT COUNT(*) as count FROM tasks WHERE status='pending'")
    
    robot_count = robots[0]['count'] if robots and robots[0]['count'] else 0
    drone_count = drones[0]['count'] if drones and drones[0]['count'] else 0
    collected_count = collected[0]['count'] if collected and collected[0]['count'] else 0
    pending_count = pending[0]['count'] if pending and pending[0]['count'] else 0
    
    # Get latest timestamp
    latest = query_db("SELECT timestamp FROM battery ORDER BY timestamp DESC LIMIT 1")
    last_update = latest[0]['timestamp'] if latest else "N/A"
    
    # Calculate runtime from server start
    elapsed = datetime.now() - SERVER_START_TIME
    hours = int(elapsed.total_seconds() // 3600)
    minutes = int((elapsed.total_seconds() % 3600) // 60)
    runtime_str = f"{hours}h {minutes}min"
    
    return {
        "runtime": runtime_str,
        "robots": robot_count,
        "drones": drone_count,
        "trash_collected": collected_count,
        "open_tasks": pending_count,
        "last_update": last_update
    }


# In-memory map patch storage for first prototype.
# Later this can be moved into SQLite.
MAP_PATCHES: List[Dict[str, Any]] = []

# Accumulated local grid-map-style state for dashboard visualization.
# This is still a lightweight prototype, not yet grid_map_msgs/GridMap.
GLOBAL_MAP = {
    "frame_id": "map",
    "width_m": 5.0,
    "height_m": 3.0,
    "resolution": 0.10,
    "rows": 30,
    "cols": 50,
    "cells": {},
}


def update_global_map_from_patch(patch: Dict[str, Any]):
    """
    Merge changed cells from a map patch into the accumulated dashboard map.
    Cells are stored by "row,col".
    """
    GLOBAL_MAP["frame_id"] = patch.get("frame_id", GLOBAL_MAP["frame_id"])
    GLOBAL_MAP["last_update"] = patch.get("timestamp", datetime.now().timestamp())

    for cell in patch.get("updated_cells", []):
        row = cell.get("row")
        col = cell.get("col")

        if row is None or col is None:
            continue

        key = f"{int(row)},{int(col)}"

        previous = GLOBAL_MAP["cells"].get(key, {})
        previous.update(cell)

        GLOBAL_MAP["cells"][key] = previous




@app.post("/api/map_patch")
def post_map_patch(patch: Dict[str, Any]):
    """
    Receive a map patch from ROS2/backend bridge.

    First prototype:
    - stores patches in memory
    - keeps only the latest 100 patches
    """
    patch["received_at"] = datetime.now().isoformat()

    MAP_PATCHES.append(patch)
    update_global_map_from_patch(patch)

    if len(MAP_PATCHES) > 100:
        del MAP_PATCHES[:-100]

    return {
        "status": "ok",
        "stored_patches": len(MAP_PATCHES),
        "latest_patch": patch,
    }


@app.get("/api/map_patch/latest")
def get_latest_map_patch():
    """
    Return the latest map patch.
    """
    if not MAP_PATCHES:
        return {
            "status": "empty",
            "message": "No map patch received yet.",
            "patch": None,
        }

    return {
        "status": "ok",
        "patch": MAP_PATCHES[-1],
    }



@app.get("/api/global_map/latest")
def get_latest_global_map():
    """
    Return the accumulated local grid-map-style state.
    """
    return {
        "status": "ok",
        "map": GLOBAL_MAP,
    }


@app.get("/api/map_patches")
def get_map_patches(limit: int = 20):
    """
    Return recent map patches.
    """
    return {
        "status": "ok",
        "count": min(limit, len(MAP_PATCHES)),
        "patches": MAP_PATCHES[-limit:],
    }


# Mount static files - this serves HTML, CSS, JS from the current directory


# --- OCTOPUS CAMERA GRID PIPELINE ROUTES ---
import subprocess as _octopus_pipeline_subprocess
from datetime import datetime as _octopus_pipeline_datetime
import time


OCTOPUS_ROOT = os.getenv(
    "OCTOPUS_ROOT",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
)


def _octopus_run_local_pipeline_command(command: str, timeout: int = 15, env=None):
    try:
        result = _octopus_pipeline_subprocess.run(
            command,
            shell=True,
            cwd=OCTOPUS_ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
            executable="/bin/bash",
            # Zusatz-Env, nicht Ersatz: os.environ muss erhalten bleiben, sonst
            # fehlen PATH und HOME und das Skript findet weder bash noch venv.
            env=None if env is None else {**os.environ, **env},
        )

        return {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
            "timestamp": _octopus_pipeline_datetime.now().isoformat(),
        }

    except _octopus_pipeline_subprocess.TimeoutExpired:
        return {
            "ok": False,
            "returncode": -1,
            "stdout": "",
            "stderr": "Command timed out",
            "timestamp": _octopus_pipeline_datetime.now().isoformat(),
        }

    except Exception as exc:
        return {
            "ok": False,
            "returncode": -1,
            "stdout": "",
            "stderr": str(exc),
            "timestamp": _octopus_pipeline_datetime.now().isoformat(),
        }


@app.get("/api/pipeline/status")
def octopus_pipeline_status():
    result = _octopus_run_local_pipeline_command(
        "./scripts/octopus_camera_grid_pipeline_status.sh",
        timeout=8,
    )

    stdout = result.get("stdout", "")

    all_running = (
        "grid_map_builder=running" in stdout
        and "map_patch_backend_bridge=running" in stdout
        and "camera_marker_transform=running" in stdout
    )

    any_running = "=running" in stdout

    if all_running:
        status = "pipeline_running"
    elif any_running:
        status = "pipeline_partial"
    else:
        status = "pipeline_stopped"

    return {
        "status": status,
        "local": result,
    }


@app.post("/api/pipeline/start")
def octopus_pipeline_start():
    result = _octopus_run_local_pipeline_command(
        "./scripts/octopus_start_camera_grid_pipeline.sh",
        timeout=25,
    )

    status = (
        "pipeline_started"
        if result["ok"] and "camera_grid_pipeline_started" in result["stdout"]
        else "pipeline_failed"
    )

    return {
        "status": status,
        "local": result,
    }


@app.post("/api/pipeline/stop")
def octopus_pipeline_stop():
    result = _octopus_run_local_pipeline_command(
        "./scripts/octopus_stop_camera_grid_pipeline.sh",
        timeout=15,
    )

    status = (
        "pipeline_stopped"
        if result["ok"] and "camera_grid_pipeline_stopped" in result["stdout"]
        else "pipeline_stop_failed"
    )

    return {
        "status": status,
        "local": result,
    }


@app.get("/api/pipeline/logs")
def octopus_pipeline_logs():
    command = """
    echo '--- grid_map_builder ---'
    tail -50 /tmp/octopus_grid_map_builder.log 2>/dev/null || true
    echo
    echo '--- map_patch_backend_bridge ---'
    tail -50 /tmp/octopus_map_patch_backend_bridge.log 2>/dev/null || true
    echo
    echo '--- camera_marker_transform ---'
    tail -80 /tmp/octopus_camera_marker_transform.log 2>/dev/null || true
    """

    result = _octopus_run_local_pipeline_command(command, timeout=8)

    return {
        "status": "ok" if result["ok"] else "failed",
        "logs": result["stdout"],
        "local": result,
    }
# --- END OCTOPUS CAMERA GRID PIPELINE ROUTES ---

# --- OCTOPUS DETECTOR ROUTES ---
# Der YOLO-Detektor auf diesem Rechner, Terminal 3 aus docs/SETUP.md. Laeuft
# lokal, nicht auf der Pi, und braucht das venv unter detect-and-localize --
# deshalb die Skripte statt eines direkten Aufrufs von hier.
DETECTOR_LOG = "/tmp/octopus_detector.log"


# --- DETEKTOR-EINSTELLUNGEN ---
# Modell, Schwelle und Tracker-Parameter. Der Detektor liest sie alle EINMAL
# beim Start und gibt sie in die Pipeline -- sie wirken also erst mit dem
# naechsten Start des Detektors, nicht sofort. Das Dashboard sagt das auch so.
#
# Die Werte gehen als DETECTOR_*-Umgebungsvariablen an
# scripts/octopus_start_detector.sh, das sie ohnehin schon liest.
DETECTOR_MODELS_DIR = os.path.join(OCTOPUS_ROOT, "detect-and-localize", "data", "models")

DETECTOR_CONFIG: Dict[str, Any] = {
    "model": "data/models/best_model_10_08_26.pt",
    "thresh": 0.60,
    "confirm_frames": 3,
    "max_lost": 5,
    "jpeg_quality": 80,
    # Cheat fuer Vorfuehrungen. 0.0 = aus, also die echten Werte.
    "confidence_bonus": 0.0,
}

# (Minimum, Maximum, Typ) je Feld. Ausserhalb wird geklemmt, nicht abgelehnt:
# ein Regler, der stumm nichts tut, ist schlimmer als einer, der begrenzt.
DETECTOR_LIMITS = {
    "thresh": (0.05, 0.95, float),
    "confirm_frames": (1, 30, int),
    "max_lost": (1, 60, int),
    "jpeg_quality": (1, 100, int),
    # Negativ erlaubt: derselbe Regler taugt dann auch, um eine Demo
    # absichtlich pessimistischer aussehen zu lassen.
    "confidence_bonus": (-0.95, 0.95, float),
}


def _detector_available_models():
    """Die Modelle, die tatsaechlich auf der Platte liegen.

    Nur diese sind waehlbar. Das ist nicht nur Komfort: der Wert landet als
    Umgebungsvariable in einem Shell-Aufruf, und eine Auswahlliste aus
    vorhandenen Dateien laesst dort nichts Fremdes durch.
    """
    try:
        names = sorted(
            name for name in os.listdir(DETECTOR_MODELS_DIR)
            if name.endswith((".pt", ".onnx")) and
            os.path.isfile(os.path.join(DETECTOR_MODELS_DIR, name))
        )
    except OSError:
        return []
    return [f"data/models/{name}" for name in names]


def _detector_config_public():
    config = dict(DETECTOR_CONFIG)
    config["available_models"] = _detector_available_models()
    config["model_exists"] = config["model"] in config["available_models"]
    config["needs_restart_to_apply"] = True
    return config


@app.get("/api/detector/config")
def get_detector_config():
    return {"status": "ok", "config": _detector_config_public()}


@app.post("/api/detector/config")
def post_detector_config(payload: Dict[str, Any]):
    rejected = {}

    for key, (low, high, cast) in DETECTOR_LIMITS.items():
        if key not in payload:
            continue
        try:
            DETECTOR_CONFIG[key] = max(low, min(high, cast(payload[key])))
        except (TypeError, ValueError):
            rejected[key] = payload[key]

    if "model" in payload:
        model = str(payload["model"])
        # Nur aus der Liste vorhandener Modelle. Ein Pfad von aussen wuerde
        # sonst ungeprueft in den Shell-Aufruf des Startskripts wandern.
        if model in _detector_available_models():
            DETECTOR_CONFIG["model"] = model
        else:
            rejected["model"] = model

    return {
        "status": "ok" if not rejected else "partial",
        "rejected": rejected,
        "config": _detector_config_public(),
    }


def _detector_start_env():
    return {
        "DETECTOR_MODEL": str(DETECTOR_CONFIG["model"]),
        "DETECTOR_THRESH": f"{float(DETECTOR_CONFIG['thresh']):.3f}",
        "DETECTOR_CONFIRM_FRAMES": str(int(DETECTOR_CONFIG["confirm_frames"])),
        "DETECTOR_MAX_LOST": str(int(DETECTOR_CONFIG["max_lost"])),
        "DETECTOR_JPEG_QUALITY": str(int(DETECTOR_CONFIG["jpeg_quality"])),
    }


@app.get("/api/detector/status")
def octopus_detector_status():
    result = _octopus_run_local_pipeline_command(
        "./scripts/octopus_detector_status.sh",
        timeout=8,
    )

    stdout = result.get("stdout", "")

    if "detector_running" in stdout:
        # YOLO braucht nach dem Start ein paar Sekunden. Ohne diese
        # Unterscheidung sieht ein voellig normaler Start wie ein haengender
        # Detektor aus.
        status = "detector_loading" if "model=loading" in stdout else "detector_running"
    elif "detector_not_running" in stdout:
        status = "detector_stopped"
    else:
        status = "detector_unknown"

    return {
        "status": status,
        "local": result,
    }


@app.post("/api/detector/start")
def octopus_detector_start():
    # 25 s, weil das Skript drei Sekunden wartet, bevor es den Prozess prueft --
    # es wartet bewusst NICHT darauf, dass YOLO fertig geladen hat. Den
    # Ladezustand holt sich das Dashboard danach ueber /api/detector/status.
    result = _octopus_run_local_pipeline_command(
        "./scripts/octopus_start_detector.sh",
        timeout=25,
        env=_detector_start_env(),
    )

    status = (
        "detector_started"
        if result["ok"] and "detector_started" in result["stdout"]
        else "detector_failed"
    )

    return {
        "status": status,
        "local": result,
    }


@app.post("/api/detector/stop")
def octopus_detector_stop():
    result = _octopus_run_local_pipeline_command(
        "./scripts/octopus_stop_detector.sh",
        timeout=20,
    )

    status = (
        "detector_stopped"
        if result["ok"] and (
            "detector_stopped" in result["stdout"]
            or "detector_not_running" in result["stdout"]
        )
        else "detector_stop_failed"
    )

    return {
        "status": status,
        "local": result,
    }


@app.get("/api/detector/log")
def octopus_detector_log():
    result = _octopus_run_local_pipeline_command(
        f"tail -80 {DETECTOR_LOG}",
        timeout=8,
    )

    return {
        "status": "ok" if result["ok"] else "failed",
        "log": result["stdout"],
        "local": result,
    }
# --- END OCTOPUS DETECTOR ROUTES ---


# --- OCTOPUS SYSTEM STOP (ABORT BUTTON) ---
# Der Abort-Knopf im Dashboard fuehrt genau den Befehl aus, den man sonst von
# Hand tippt:
#
#   OCTOPUS_STOP_EVE=true ./scripts/stop_octopus_debug_stack.sh
#
# Also: Backend, ROS-Nodes, rosbridge und Detektor auf diesem Rechner, dazu
# Kamera und PX4-Bruecke auf der Pi.
#
# Der Aufruf darf NICHT auf das Skript warten: das Skript beendet unter anderem
# dieses Backend, die Antwort auf diesen Request kaeme also nie an. Deshalb wird
# es abgekoppelt gestartet, mit einer Sekunde Vorlauf, damit die Antwort noch
# rausgeht -- und der Browser weiss, dass sein Klick angekommen ist, statt einen
# Netzwerkfehler zu sehen.
OCTOPUS_STOP_SCRIPT = "./scripts/stop_octopus_debug_stack.sh"
OCTOPUS_STOP_GRACE_SEC = 1


@app.post("/api/system/stop")
def octopus_system_stop():
    env = dict(os.environ)
    env["OCTOPUS_STOP_EVE"] = "true"

    command = f"sleep {OCTOPUS_STOP_GRACE_SEC}; exec {OCTOPUS_STOP_SCRIPT}"

    try:
        _octopus_pipeline_subprocess.Popen(
            ["/bin/bash", "-c", command],
            cwd=OCTOPUS_ROOT,
            env=env,
            # start_new_session, sonst nimmt das Skript beim Beenden dieses
            # Backends sein eigenes Sterben mit -- es haengt an derselben
            # Prozessgruppe und wuerde vor dem Rest abgeschossen.
            start_new_session=True,
            stdout=open("/tmp/octopus_system_stop.log", "ab", buffering=0),
            stderr=_octopus_pipeline_subprocess.STDOUT,
            stdin=_octopus_pipeline_subprocess.DEVNULL,
        )
    except Exception as exc:
        return {
            "status": "stop_failed",
            "error": str(exc),
            "timestamp": _octopus_pipeline_datetime.now().isoformat(),
        }

    return {
        "status": "stopping",
        "stops_eve": True,
        "grace_sec": OCTOPUS_STOP_GRACE_SEC,
        "log": "/tmp/octopus_system_stop.log",
        "timestamp": _octopus_pipeline_datetime.now().isoformat(),
    }
# --- END OCTOPUS SYSTEM STOP ---



CAMERA_TRANSFORM_STATUS = {
    "mode": "apriltag_field_homography",
    "state": "unknown",
    "has_homography": False,
    "is_stale": False,
    "is_transform_allowed": False,
    "homography_age_sec": None,
    "detected_marker_ids": [],
    "missing_marker_ids": [],
    "required_marker_ids": [61, 65, 57, 11],
    "backend_received_at": None,
}


@app.post("/api/camera_transform/status")
def post_camera_transform_status(payload: Dict[str, Any]):
    global CAMERA_TRANSFORM_STATUS

    CAMERA_TRANSFORM_STATUS = dict(payload)
    CAMERA_TRANSFORM_STATUS["backend_received_at"] = time.time()

    return {
        "status": "ok",
        "camera_transform_status": CAMERA_TRANSFORM_STATUS,
    }


@app.get("/api/camera_transform/status")
def get_camera_transform_status():
    return {
        "status": "ok",
        "camera_transform_status": CAMERA_TRANSFORM_STATUS,
    }


# --- Local Camera Grid API ---
latest_local_camera_grid_patch = None

@app.post("/api/local_camera_grid")
async def receive_local_camera_grid_patch(patch: dict):
    global latest_local_camera_grid_patch
    latest_local_camera_grid_patch = {
        "status": "ok",
        "patch": patch,
        "received_at": datetime.now().isoformat(),
    }
    return {"status": "ok"}

@app.get("/api/local_camera_grid/latest")
async def get_latest_local_camera_grid_patch():
    if latest_local_camera_grid_patch is None:
        return {
            "status": "empty",
            "patch": None,
            "received_at": None,
        }
    return latest_local_camera_grid_patch

app.mount("/", StaticFiles(directory=".", html=True), name="static")
