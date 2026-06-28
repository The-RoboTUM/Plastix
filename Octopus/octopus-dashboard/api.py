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

    LATEST_CAMERA_DEBUG["image"] = {
        "received_at": received_at,
        "format": image_format,
        "stamp": payload.get("stamp"),
        "frame_id": payload.get("frame_id", "camera"),
        "data_url": data_url,
    }
    LATEST_CAMERA_DEBUG["received_at"] = received_at

    return {
        "status": "ok",
        "received_at": received_at,
        "has_image": bool(data_url),
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
        "detections": LATEST_CAMERA_DEBUG["detections"],
    }


@app.get("/api/camera_debug/latest")
def get_latest_camera_debug():
    return {
        "status": "ok" if (LATEST_CAMERA_DEBUG["image"] or LATEST_CAMERA_DEBUG["detections"]) else "empty",
        "received_at": LATEST_CAMERA_DEBUG["received_at"],
        "image": LATEST_CAMERA_DEBUG["image"],
        "detections": LATEST_CAMERA_DEBUG["detections"],
    }
# --- END CAMERA DEBUG / DETECTION INSPECTOR ROUTES ---


# --- OCTOPUS EVE CAMERA ROUTES ---
import subprocess
from datetime import datetime


EVE_SSH_TARGET = "eve-pi"


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
    result = _octopus_run_eve_command("~/octopus_camera_status.sh", timeout=8)

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
    result = _octopus_run_eve_command("~/octopus_start_camera.sh", timeout=15)

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
    result = _octopus_run_eve_command("~/octopus_stop_camera.sh", timeout=10)

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
    result = _octopus_run_eve_command("tail -80 /tmp/octopus_camera_node.log", timeout=8)

    return {
        "status": "ok" if result["ok"] else "failed",
        "log": result["stdout"],
        "ssh": result,
    }
# --- END OCTOPUS EVE CAMERA ROUTES ---



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


OCTOPUS_ROOT = "/home/dominik/projects/PlastiX/Octopus"


def _octopus_run_local_pipeline_command(command: str, timeout: int = 15):
    try:
        result = _octopus_pipeline_subprocess.run(
            command,
            shell=True,
            cwd=OCTOPUS_ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
            executable="/bin/bash",
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
