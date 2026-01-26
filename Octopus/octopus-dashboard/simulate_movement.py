import sqlite3
import time
import random
from datetime import datetime
import math
import requests
import json
import os

DB_PATH = "octopusfinal.db"
LOCATION_CACHE_FILE = "laptop_location.json"

# Local coordinate system
class LocalCoords:
    def __init__(self, origin_lat, origin_lon):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.R = 6371000
        self.lat_factor = math.radians(1) * self.R
        self.lon_factor = math.radians(1) * self.R * math.cos(math.radians(origin_lat))
    
    def gps_to_local(self, lat, lon):
        dlat = lat - self.origin_lat
        dlon = lon - self.origin_lon
        y = dlat * self.lat_factor
        x = dlon * self.lon_factor
        return (x, y)
    
    def local_to_gps(self, x, y):
        dlat = y / self.lat_factor
        dlon = x / self.lon_factor
        return (self.origin_lat + dlat, self.origin_lon + dlon)

def get_laptop_location():
    """
    Automatically get laptop's current GPS position.
    Tries multiple methods:
    1. Load from cache (if recently updated)
    2. Get from IP geolocation API
    3. Prompt user for manual input
    """
    
    # Try to load from cache first
    if os.path.exists(LOCATION_CACHE_FILE):
        try:
            with open(LOCATION_CACHE_FILE, 'r') as f:
                cache = json.load(f)
                age_minutes = (time.time() - cache['timestamp']) / 60
                if age_minutes < 30:  # Cache valid for 30 minutes
                    print(f"📍 Using cached laptop location (age: {age_minutes:.1f} min)")
                    print(f"   Lat: {cache['lat']}, Lon: {cache['lon']}, Location: {cache.get('city', 'Unknown')}")
                    return cache['lat'], cache['lon']
        except Exception as e:
            print(f"⚠️  Could not read cache: {e}")
    
    # Try IP-based geolocation
    print("🌐 Detecting laptop location from IP address...")
    try:
        # Using ipapi.co - free, no API key required
        response = requests.get('https://ipapi.co/json/', timeout=5)
        if response.status_code == 200:
            data = response.json()
            lat = data.get('latitude')
            lon = data.get('longitude')
            city = data.get('city', 'Unknown')
            country = data.get('country_name', 'Unknown')
            
            if lat and lon:
                print(f"✅ Location detected: {city}, {country}")
                print(f"   Lat: {lat}, Lon: {lon}")
                
                # Save to cache
                cache_data = {
                    'lat': lat,
                    'lon': lon,
                    'city': city,
                    'country': country,
                    'timestamp': time.time()
                }
                with open(LOCATION_CACHE_FILE, 'w') as f:
                    json.dump(cache_data, f)
                
                return lat, lon
    except Exception as e:
        print(f"⚠️  Auto-detection failed: {e}")
    
    # Fallback: Ask user for manual input
    print("\n❌ Could not auto-detect location.")
    print("Please enter your laptop's GPS coordinates:")
    try:
        lat = float(input("Latitude (e.g., 49.451): "))
        lon = float(input("Longitude (e.g., 11.076): "))
        
        # Save to cache
        cache_data = {
            'lat': lat,
            'lon': lon,
            'city': 'Manual Entry',
            'timestamp': time.time()
        }
        with open(LOCATION_CACHE_FILE, 'w') as f:
            json.dump(cache_data, f)
        
        return lat, lon
    except Exception as e:
        print(f"⚠️  Invalid input: {e}")
        print("Using default location (Nuremberg)")
        return 49.451, 11.076

# Get laptop position at startup
LAPTOP_LAT, LAPTOP_LON = get_laptop_location()
coords = LocalCoords(LAPTOP_LAT, LAPTOP_LON)

# Define devices with initial LOCAL positions (in meters from laptop)
# These will automatically spawn around Munich when laptop is detected there
devices = {
    "drone_1": {
        "type": "drone",
        "x": 50,     # 50m East from laptop (Munich center area)
        "y": 30,     # 30m North
        "alt": 20.5,
        "battery": 87,
        "state": "active",
        "speed": 25,  # meters per update
        "battery_drain": 0.5
    },
    "robot_1": {
        "type": "robot",
        "x": -100,   # 100m West (around Marienplatz area)
        "y": -80,    # 80m South
        "alt": 0.2,
        "battery": 65,
        "state": "active",
        "speed": 12,
        "battery_drain": 0.3
    },
    "robot_2": {
        "type": "robot",
        "x": 150,    # 150m East (Viktualienmarkt area)
        "y": 120,    # 120m North
        "alt": 0.3,
        "battery": 92,
        "state": "active",
        "speed": 18,
        "battery_drain": 0.4
    },
    "robot_3": {
        "type": "robot",
        "x": -200,   # 200m West (English Garden direction)
        "y": 180,    # 180m North
        "alt": 0.2,
        "battery": 78,
        "state": "active",
        "speed": 15,
        "battery_drain": 0.35
    },
    "robot_4": {
        "type": "robot",
        "x": 220,    # 220m East (Isar direction)
        "y": -150,   # 150m South
        "alt": 0.2,
        "battery": 99,
        "state": "active",
        "speed": 20,
        "battery_drain": 0.45
    }
}

def update_device_position(device_id, device_data):
    """Move device in local coordinate system"""
    angle = random.uniform(0, 2 * math.pi)
    speed = device_data["speed"]
    
    # Update position in local coordinates
    device_data["x"] += speed * math.cos(angle)
    device_data["y"] += speed * math.sin(angle)
    
    # Keep within reasonable bounds (1km radius from laptop)
    distance = math.sqrt(device_data["x"]**2 + device_data["y"]**2)
    if distance > 1000:  # 1km limit
        # Push back towards center
        angle_back = math.atan2(device_data["y"], device_data["x"]) + math.pi
        device_data["x"] += 50 * math.cos(angle_back)
        device_data["y"] += 50 * math.sin(angle_back)
    
    # Drone altitude variation
    if device_data["type"] == "drone":
        device_data["alt"] += random.uniform(-1, 1)
        device_data["alt"] = max(15, min(30, device_data["alt"]))

def update_battery(device_id, device_data):
    """Decrease battery and update state"""
    device_data["battery"] -= device_data["battery_drain"]
    
    if device_data["battery"] <= 0:
        device_data["battery"] = 0
        device_data["state"] = "idle"
    elif device_data["battery"] <= 20:
        device_data["state"] = "charging"
        device_data["battery"] += 2.0
    elif device_data["battery"] >= 95:
        device_data["state"] = "active"
    
    device_data["battery"] = max(0, min(100, device_data["battery"]))

def save_to_database():
    """Save all device data to database (converting local to GPS)"""
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    now = datetime.utcnow().isoformat()
    
    # Clear old data
    cur.execute("DELETE FROM locations")
    cur.execute("DELETE FROM battery")
    
    # Save LAPTOP position first (origin point)
    cur.execute("""
        INSERT INTO locations (origin_id, type, latitude, longitude, altitude, timestamp)
        VALUES (?, ?, ?, ?, ?, ?)
    """, ("laptop", "laptop", LAPTOP_LAT, LAPTOP_LON, 0, now))
    
    cur.execute("""
        INSERT INTO battery (device_id, origin_id, type, latitude, longitude, altitude, battery_percent, state, timestamp)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, ("laptop", "laptop", "laptop", LAPTOP_LAT, LAPTOP_LON, 0, 100, "active", now))
    
    # Insert updated data for all devices
    for device_id, data in devices.items():
        # Convert local coords to GPS
        lat, lon = coords.local_to_gps(data["x"], data["y"])
        
        # Locations
        cur.execute("""
            INSERT INTO locations (origin_id, type, latitude, longitude, altitude, timestamp)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (device_id, data["type"], lat, lon, data["alt"], now))
        
        # Battery
        cur.execute("""
            INSERT INTO battery (device_id, origin_id, type, latitude, longitude, altitude, battery_percent, state, timestamp)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (device_id, device_id, data["type"], lat, lon, data["alt"], 
              data["battery"], data["state"], now))
    
    conn.commit()
    conn.close()

def print_status():
    """Print current status with local coordinates"""
    print("\n" + "="*80)
    print(f"Update at {datetime.now().strftime('%H:%M:%S')}")
    print(f"💻 Laptop (Origin): ({LAPTOP_LAT:.6f}, {LAPTOP_LON:.6f}) - Garching")
    print("="*80)
    
    for device_id, data in devices.items():
        state_emoji = "🟢" if data["state"] == "active" else "🔋" if data["state"] == "charging" else "⚫"
        distance = math.sqrt(data["x"]**2 + data["y"]**2)
        bearing = math.degrees(math.atan2(data["x"], data["y"])) % 360
        
        print(f"{state_emoji} {device_id:12} | Battery: {data['battery']:5.1f}% | "
              f"Local: ({data['x']:6.1f}m, {data['y']:6.1f}m) | "
              f"Dist: {distance:6.1f}m | Bear: {bearing:5.1f}°")

def main():
    print("🤖 Starting Octopus Dashboard Simulation (Auto-Location)")
    print("="*80)
    print(f"📍 Laptop Origin: {LAPTOP_LAT:.6f}, {LAPTOP_LON:.6f}")
    print(f"💾 Location cached in: {LOCATION_CACHE_FILE}")
    print("   (Delete this file to force location refresh)")
    print("="*80)
    print("Press Ctrl+C to stop\n")
    
    update_count = 0
    
    try:
        while True:
            # Update all devices
            for device_id, device_data in devices.items():
                if device_data["state"] == "active":
                    update_device_position(device_id, device_data)
                update_battery(device_id, device_data)
            
            # Save to database
            save_to_database()
            
            # Print status
            print_status()
            
            update_count += 1
            
            # Wait before next update
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation stopped")
        print(f"Total updates: {update_count}")

if __name__ == "__main__":
    main()