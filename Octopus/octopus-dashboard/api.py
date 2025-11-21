
from fastapi import FastAPI
import sqlite3
from pydantic import BaseModel
from typing import List
import os

DB_PATH = os.getenv("DB_PATH", "octopus.db")
app = FastAPI()

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
    # Wenn du eine tasks-Tabelle hast, lies daraus; für now return sample
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
    # basic computed stats; expand as needed
    total_robots = 3
    total_drones = 1
    collected = 24
    return {"runtime":"2h 13min","robots":total_robots,"drones":total_drones,"trash_collected":collected,"open_tasks":1,"last_update":"2025-11-13T12:06:00Z"}
