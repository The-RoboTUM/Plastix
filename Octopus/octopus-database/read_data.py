import sqlite3

conn = sqlite3.connect("octopus.db")
cur = conn.cursor()

print("📍 Stored Locations:")
for row in cur.execute("SELECT * FROM locations"):
    print(row)

conn.close()
