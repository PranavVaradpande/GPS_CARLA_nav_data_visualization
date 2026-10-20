import carla
import numpy as np
import socket
import json
import math
import time
from db_sql import CarlaDatabaseLogger

HOST = 'localhost'
PORT_CONTROLLER = 5556

# Stanley Controller & Speed Parameters
k_e = 0.5          # Cross-track error gain
k_v = 0.4          # Lookahead speed scaling factor
L_min = 2.5        # Minimum lookahead distance (m)
L_max = 12.0       # Maximum lookahead distance (m)
TARGET_SPEED_MAX = 35.0  # Max target speed on straights (km/h)
TARGET_SPEED_MIN = 12.0  # Min target speed in sharp turns (km/h)

class StanleyController:
    @staticmethod
    def compute_steering(vehicle_transform, vehicle_velocity, target_wp, next_wp):
        v_x = vehicle_transform.location.x
        v_y = vehicle_transform.location.y
        yaw = math.radians(vehicle_transform.rotation.yaw)
        v_speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2)  # m/s

        # Path segment heading
        dx = next_wp[0] - target_wp[0]
        dy = next_wp[1] - target_wp[1]
        path_yaw = math.atan2(dy, dx)

        # 1. Heading Error
        heading_error = path_yaw - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 2. Cross-Track Error
        vec_wp_veh = np.array([v_x - target_wp[0], v_y - target_wp[1]])
        cross_track_error = vec_wp_veh[0] * math.sin(path_yaw) - vec_wp_veh[1] * math.cos(path_yaw)

        # 3. Stanley Steering Law
        crosstrack_term = math.atan2(k_e * cross_track_error, v_speed + 0.1)
        steer_rad = heading_error + crosstrack_term
        steer = math.atan2(math.sin(steer_rad), math.cos(steer_rad)) / (math.pi / 4)
        return float(np.clip(steer, -1.0, 1.0))

def fetch_waypoint_chunk(command="FETCH_NEXT"):
    """Retries connection until RRT* planner server responds."""
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT_CONTROLLER))
            sock.sendall(json.dumps({"command": command}).encode('utf-8'))
            sock.shutdown(socket.SHUT_WR)

            data = b""
            while True:
                packet = sock.recv(4096)
                if not packet:
                    break
                data += packet
            sock.close()

            if data:
                res = json.loads(data.decode('utf-8'))
                return res.get("waypoints", []), res.get("is_last_chunk", False)
        except Exception:
            time.sleep(1.0)

def compute_curvature(p1, p2, p3):
    """Calculates local curvature kappa for adaptive speed reduction."""
    v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
    norm1, norm2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if norm1 < 1e-3 or norm2 < 1e-3:
        return 0.0
    angle = math.acos(np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0))
    return angle / (norm1 + norm2)

def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    carla_map = world.get_map()

    # Locate Ego Vehicle
    ego_vehicle = None
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'hero':
            ego_vehicle = actor
            break

    if not ego_vehicle:
        print("[Controller] Hero vehicle not found! Run `GPS_teleop.py` first.")
        return

    # Initialize Database Logger
    db = CarlaDatabaseLogger()
    trip_id = db.get_latest_trip_id()
    if not trip_id:
        print("[Controller] No active trip found in DB. Defaulting to trip_id = 1.")
        trip_id = 1

    print("[Controller] Connected to Vehicle. Fetching initial waypoints...")
    active_waypoints, is_last_chunk = fetch_waypoint_chunk("FETCH_FIRST")

    if not active_waypoints:
        print("[Controller] No waypoints received from planner. Exiting.")
        return

    current_idx = 0
    destination_reached = False
    start_time = time.time()

    print(f"[Controller] Driving active for Trip ID: {trip_id}. Following path and logging telemetry...")

    try:
        while not destination_reached:
            elapsed_time = time.time() - start_time
            transform = ego_vehicle.get_transform()
            velocity = ego_vehicle.get_velocity()

            # Speed Calculations
            v_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            speed_kmh = 3.6 * v_ms
            curr_pos = [transform.location.x, transform.location.y, transform.location.z]
            curr_geo = carla_map.transform_to_geolocation(transform.location)

            # 1. Adaptive Lookahead Distance
            lookahead_dist = float(np.clip(k_v * v_ms + L_min, L_min, L_max))

            # Target Waypoint Tracking
            while current_idx < len(active_waypoints) - 1:
                wp = active_waypoints[current_idx]
                dist = math.hypot(wp[0] - curr_pos[0], wp[1] - curr_pos[1])
                if dist >= lookahead_dist:
                    break
                current_idx += 1

            target_wp = active_waypoints[current_idx]
            next_wp = active_waypoints[min(current_idx + 1, len(active_waypoints) - 1)]

            # 2. Batch Requesting (Fetch Next 50 Waypoints)
            remaining_wps = len(active_waypoints) - current_idx
            if remaining_wps < 15 and not is_last_chunk:
                print("[Controller] Requesting next 50 waypoints...")
                new_wps, is_last_chunk = fetch_waypoint_chunk("FETCH_NEXT")
                if new_wps:
                    active_waypoints.extend(new_wps)

            # 3. Destination Distance & ETA
            final_wp = active_waypoints[-1]
            dist_to_final = math.hypot(final_wp[0] - curr_pos[0], final_wp[1] - curr_pos[1])
            eta_seconds = dist_to_final / (v_ms + 1e-3)

            # 4. Stream Telemetry to SQLite Database
            db.log_telemetry(
                trip_id=trip_id,
                elapsed_sec=round(elapsed_time, 2),
                curr_xyz=curr_pos,
                curr_gps=[curr_geo.latitude, curr_geo.longitude],
                speed_kmh=round(speed_kmh, 2),
                lookahead_m=round(lookahead_dist, 2),
                dist_goal_m=round(dist_to_final, 2),
                eta_sec=round(eta_seconds, 1)
            )

            # 5. Check Destination Reached
            if is_last_chunk and dist_to_final < 2.0:
                print(f"[Controller] Destination Reached (Dist: {dist_to_final:.2f}m)! Applying full brakes.")
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = True
                ego_vehicle.apply_control(control)
                db.end_trip(trip_id, status='COMPLETED')
                destination_reached = True
                break

            # 6. Adaptive Speed Control based on Local Curvature
            p1 = active_waypoints[max(0, current_idx - 1)]
            curvature = compute_curvature(p1, target_wp, next_wp)
            target_speed = np.clip(TARGET_SPEED_MAX / (1.0 + 15.0 * curvature), TARGET_SPEED_MIN, TARGET_SPEED_MAX)

            # 7. Compute Steering Angle (Stanley Controller)
            steer = StanleyController.compute_steering(transform, velocity, target_wp, next_wp)

            # 8. Compute Throttle & Brake
            speed_error = (target_speed - speed_kmh) / TARGET_SPEED_MAX
            control = carla.VehicleControl()
            control.steer = steer

            if speed_error > 0:
                control.throttle = float(np.clip(speed_error * 1.5, 0.0, 0.75))
                control.brake = 0.0
            else:
                control.throttle = 0.0
                control.brake = float(np.clip(-speed_error * 1.2, 0.0, 1.0))

            # 9. APPLY CONTROL COMMAND TO VEHICLE IN CARLA
            ego_vehicle.apply_control(control)

            world.tick()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("[Controller] Manual interruption.")
        db.end_trip(trip_id, status='INTERRUPTED')

    finally:
        stop_control = carla.VehicleControl()
        stop_control.throttle = 0.0
        stop_control.brake = 1.0
        ego_vehicle.apply_control(stop_control)

if __name__ == '__main__':
    main()