import sqlite3, folium

conn = sqlite3.connect("octopus.db")
cur = conn.cursor()

locations = cur.execute("SELECT origin_id, type, latitude, longitude FROM locations").fetchall()
conn.close()

# Center map roughly around first entry
if locations:
    center_lat = locations[0][2]
    center_lon = locations[0][3]
else:
    center_lat, center_lon = 48.137, 11.576

m = folium.Map(location=[center_lat, center_lon], zoom_start=13)

for origin_id, device_type, lat, lon in locations:
    folium.Marker([lat, lon], tooltip=f"{origin_id} ({device_type})").add_to(m)

m.save("map.html")
print("🗺️ Map created! Open map.html in your browser.")
