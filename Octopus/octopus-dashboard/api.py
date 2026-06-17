from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
import sqlite3
from pydantic import BaseModel
from typing import List, Dict, Any
import os
from datetime import datetime

DB_PATH = os.getenv("DB_PATH", "octopusfinal.db")
app = FastAPI()

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
app.mount("/", StaticFiles(directory=".", html=True), name="static")