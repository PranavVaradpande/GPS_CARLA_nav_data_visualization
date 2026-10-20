# 🚗 Modular CARLA Autonomous Vehicle Navigation & Telemetry Analytics System

A complete, modular autonomous vehicle pipeline designed for the **CARLA Simulator**. This project bypasses CARLA's built-in `GlobalRoutePlanner` to demonstrate custom algorithmic path planning, dynamic lateral vehicle control, IPC socket communication, real-time SQLite telemetry logging, and business intelligence analytics via Power BI.

---

## 🏗️ System Architecture

The pipeline operates across three independent Python processes communicating over local sockets and persisting data to a central database:

```text
+------------------------+                         +-------------------------------+
|   1. GPS_teleop.py     | ----------------------> | 2. RRTstar_mission_planner.py |
| (Pygame Map & Goal)    |                         |  (Custom Graph A* Planner)    |
+------------------------+                         +-------------------------------+
            |                                                       |
      Trip Creation                                       Chunked Waypoints (50 WPs)
            |                                                       |
            v                                                       v
+----------------------------------------------------------------------------------+
|                               3. Vehicle_control.py                              |
|                        (Stanley Steering + Throttle PID)                         |
+----------------------------------------------------------------------------------+
                                            |
                                   Telemetry Streaming
                                            |
                                            v
                              +----------------------------+
                              |   sqlite3 Database Logger  |
                              |   (carla_telemetry.db)     |
                              +----------------------------+
                                            |
                                      Snapshot Import
                                            |
                                            v
                              +----------------------------+
                              |    Power BI Dashboard      |
                              |   (DAX Analytics & KPIs)   |
                              +----------------------------+
```

---

## 🛠️ Key Features

* **Interactive Mapping & Mission Setup (`GPS_teleop.py`)**:
  * Real-time 2D Pygame visual interface with map panning, zoom, ego vehicle tracking, and goal selection.
  * Captures geolocation and local CARLA coordinates for trip initiation.

* **Custom Graph A* Mission Planner (`RRTstar_mission_planner.py`)**:
  * Pure-Python OpenDRIVE road network planner with multi-lane expansion and topological routing.
  * Streams paths in 50-waypoint chunks over non-blocking TCP sockets to handle long-distance missions efficiently.

* **Stanley Controller & Adaptive Vehicle Dynamics (`Vehicle_control.py`)**:
  * **Stanley Steering Law**: Heading error correction combined with non-linear cross-track error compensation.
  * **Adaptive Lookahead Distance**: Dynamic scaling based on vehicle velocity:
    $$\text{Lookahead} = \text{clip}(k_v \cdot v + L_{\min}, L_{\min}, L_{\max})$$
  * **Curvature-Based Speed Scaling**: Automatically reduces target velocity during sharp turns based on local path curvature $\kappa$.

* **SQLite Telemetry Logging (`db_logger.py`)**:
  * Thread-safe logging utilizing Write-Ahead Logging (WAL) mode.
  * Relational schema connecting `trips` metadata to high-frequency `telemetry_logs` (x/y/z coordinates, speed, lookahead, distance-to-goal, ETA).

* **Power BI Telemetry Analytics**:
  * Interactive dashboard evaluating vehicle performance across individual test runs.
  * 2D spatial trajectory plots, velocity profiles, controller dynamics, and trip summary KPIs using custom DAX measures.

---

## 📁 Repository Structure

```text
.
├── GPS_teleop.py              # Pygame map viewer, goal selector & trip starter
├── RRTstar_mission_planner.py # Custom Graph A* OpenDRIVE road network planner
├── Vehicle_control.py         # Stanley controller & vehicle actuation loop
├── db_logger.py               # SQLite database interface class
├── carla_telemetry.db         # SQLite database file (auto-generated on first run)
├── Telemetry_Dashboard.pbix   # Power BI report file
└── README.md                  # Project documentation
```

---

## 🗄️ Database Schema

The SQLite database (`carla_telemetry.db`) consists of two normalized tables linked by a $1 : N$ relationship on `trip_id`:

### `trips`
| Column | Type | Description |
| :--- | :--- | :--- |
| `trip_id` | `INTEGER PK` | Auto-incrementing trip identifier |
| `town_name` | `TEXT` | CARLA map loaded (e.g., `Town05`) |
| `start_x, start_y, start_z` | `REAL` | Origin coordinates in CARLA world |
| `goal_x, goal_y, goal_z` | `REAL` | Target destination coordinates |
| `start_time` | `TIMESTAMP` | Timestamp when mission was dispatched |
| `status` | `TEXT` | `IN_PROGRESS`, `COMPLETED`, or `INTERRUPTED` |

### `telemetry_logs`
| Column | Type | Description |
| :--- | :--- | :--- |
| `log_id` | `INTEGER PK` | Auto-incrementing log entry identifier |
| `trip_id` | `INTEGER FK` | References `trips(trip_id)` |
| `elapsed_time_sec` | `REAL` | Time elapsed since trip start (seconds) |
| `current_x, current_y, current_z` | `REAL` | Real-time vehicle world position |
| `speed_kmh` | `REAL` | Vehicle ground velocity (km/h) |
| `lookahead_distance_m` | `REAL` | Dynamic Stanley lookahead distance (m) |
| `distance_to_goal_m` | `REAL` | Remaining distance to target (m) |
| `eta_seconds` | `REAL` | Estimated time to arrival (seconds) |

---

## 📊 Power BI Dashboard Setup

### Key DAX Measures Included
```dax
-- Average Vehicle Speed
Avg Speed = AVERAGE(telemetry_logs[speed_kmh])

-- Peak Velocity Reached
Max Speed = MAX(telemetry_logs[speed_kmh])

-- Total Elapsed Drive Time
Trip Duration = 
VAR TotalSec = MAX(telemetry_logs[elapsed_time_sec])
VAR Mins = INT(TotalSec / 60)
VAR Secs = INT(MOD(TotalSec, 60))
RETURN FORMAT(Mins, "00") & ":" & FORMAT(Secs, "00")
```

### Visual Features
1. **Trip Slicer**: Filters dashboard visuals by specific `trip_id` test runs.
2. **2D Trajectory Map**: Scatter plot (`current_x` vs `current_y`) rendering the exact path driven by the vehicle.
3. **Controller Dynamics**: Line & Clustered Column Chart plotting velocity alongside adaptive lookahead expansion.
4. **ETA & Distance Decay**: Line chart tracking distance-to-goal convergence over elapsed time.

---

## ⚡ Quick Start & Execution Order

### Prerequisites
* **Python 3.7+**
* **CARLA Simulator** (v0.9.13+ recommended)
* Dependencies:
  ```bash
  pip install carla pygame numpy
  ```

### Launch Sequence
1. **Start CARLA Simulator**:
   Launch the CARLA server executable on port `2000`.

2. **Run Mission Planner Server**:
   ```bash
   python RRTstar_mission_planner.py
   ```

3. **Run Interactive Map Teleop**:
   ```bash
   python GPS_teleop.py
   ```
   * Click on the Pygame window to place a goal marker.
   * Press **ENTER** to confirm and start the trip.

4. **Run Vehicle Controller**:
   ```bash
   python Vehicle_control.py
   ```

5. **View Dashboard in Power BI**:
   * Open `Telemetry_Dashboard.pbix`.
   * Click **Refresh** on the Home ribbon to pull the latest telemetry logs from `carla_telemetry.db`.
