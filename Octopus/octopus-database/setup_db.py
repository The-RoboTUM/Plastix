import sqlite3

# 1. Connect (creates file if not exists)
conn = sqlite3.connect("octopus.db")
cur = conn.cursor()

# 2. Create table
cur.execute("""
CREATE TABLE IF NOT EXISTS locations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    origin_id TEXT NOT NULL,
    type TEXT CHECK (type IN ('robot', 'drone')),
    latitude REAL NOT NULL,
    longitude REAL NOT NULL,
    altitude REAL,
    timestamp TEXT NOT NULL
)
""")

conn.commit()
conn.close()
print("✅ Database 'octopus.db' created successfully.")
