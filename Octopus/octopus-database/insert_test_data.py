import sqlite3, datetime

conn = sqlite3.connect("octopus.db")
cur = conn.cursor()

test_data = [
    ("drone_1", "drone", 48.137, 11.576, 20.5, datetime.datetime.utcnow().isoformat()),
    ("robot_1", "robot", 48.138, 11.578, 0.2, datetime.datetime.utcnow().isoformat()),
    ("robot_2", "robot", 48.140, 11.580, 0.3, datetime.datetime.utcnow().isoformat())
]

cur.executemany("""
INSERT INTO locations (origin_id, type, latitude, longitude, altitude, timestamp)
VALUES (?, ?, ?, ?, ?, ?)
""", test_data)

conn.commit()
conn.close()
print("✅ Test data inserted.")
