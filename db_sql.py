import sqlite3

DB_FILE = "carla_telemetry.db"

class CarlaDatabaseLogger:
    def __init__(self, db_file=DB_FILE):
        self.db_file = db_file
        self.init_db()

    def get_connection(self):
        conn = sqlite3.connect(self.db_file, timeout=10.0)
        conn.execute("PRAGMA journal_mode=WAL;")  # Write-Ahead Logging for safety
        return conn

    def init_db(self):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS trips (
                trip_id INTEGER PRIMARY KEY AUTOINCREMENT,
                town_name TEXT NOT NULL,
                start_x REAL, start_y REAL, start_z REAL,
                start_lat REAL, start_lon REAL,
                goal_x REAL, goal_y REAL, goal_z REAL,
                goal_lat REAL, goal_lon REAL,
                start_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                end_time TIMESTAMP,
                status TEXT DEFAULT 'IN_PROGRESS'
            );
            """)
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS telemetry_logs (
                log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                trip_id INTEGER NOT NULL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                elapsed_time_sec REAL NOT NULL,
                current_x REAL, current_y REAL, current_z REAL,
                current_lat REAL, current_lon REAL,
                speed_kmh REAL NOT NULL,
                lookahead_distance_m REAL NOT NULL,
                distance_to_goal_m REAL NOT NULL,
                eta_seconds REAL NOT NULL,
                FOREIGN KEY (trip_id) REFERENCES trips(trip_id)
            );
            """)
            conn.commit()

    def start_new_trip(self, town, start_xyz, goal_xyz, start_gps, goal_gps):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO trips (
                    town_name, 
                    start_x, start_y, start_z, start_lat, start_lon,
                    goal_x, goal_y, goal_z, goal_lat, goal_lon,
                    status
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'IN_PROGRESS')
            """, (
                town,
                start_xyz[0], start_xyz[1], start_xyz[2], start_gps[0], start_gps[1],
                goal_xyz[0], goal_xyz[1], goal_xyz[2], goal_gps[0], goal_gps[1]
            ))
            trip_id = cursor.lastrowid
            conn.commit()
            print(f"[Database] Started new trip logging with Trip ID: {trip_id}")
            return trip_id

    def log_telemetry(self, trip_id, elapsed_sec, curr_xyz, curr_gps, speed_kmh, lookahead_m, dist_goal_m, eta_sec):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO telemetry_logs (
                    trip_id, elapsed_time_sec,
                    current_x, current_y, current_z,
                    current_lat, current_lon,
                    speed_kmh, lookahead_distance_m,
                    distance_to_goal_m, eta_seconds
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                trip_id, elapsed_sec,
                curr_xyz[0], curr_xyz[1], curr_xyz[2],
                curr_gps[0], curr_gps[1],
                speed_kmh, lookahead_m, dist_goal_m, eta_sec
            ))
            conn.commit()

    def get_latest_trip_id(self):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT trip_id FROM trips WHERE status = 'IN_PROGRESS' ORDER BY trip_id DESC LIMIT 1")
            row = cursor.fetchone()
            return row[0] if row else None

    def end_trip(self, trip_id, status='COMPLETED'):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                UPDATE trips 
                SET end_time = CURRENT_TIMESTAMP, status = ? 
                WHERE trip_id = ?
            """, (status, trip_id))
            conn.commit()
            print(f"[Database] Trip {trip_id} closed with status: {status}")