import carla
import pygame
import numpy as np
import socket
import json
import math
import time
import sys

WINDOW_WIDTH = 900
WINDOW_HEIGHT = 900
HOST = 'localhost'
PORT_CARLA = 2000
PORT_PLANNER = 5555

def select_town():
    towns = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town07", "Town10HD"]
    print("\n--- Available CARLA Towns ---")
    for idx, town in enumerate(towns):
        print(f"[{idx + 1}] {town}")
    
    choice = input("\nSelect Town number (1-7) [Default: 1]: ").strip()
    idx = int(choice) - 1 if choice.isdigit() and 1 <= int(choice) <= len(towns) else 0
    return towns[idx]

class InteractiveMap:
    def __init__(self, bounds, width, height):
        self.min_x, self.max_x, self.min_y, self.max_y = bounds
        self.width = width
        self.height = height
        
        # Pan & Zoom parameters
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self.drag_start = (0, 0)

    def world_to_pixel(self, x, y):
        norm_x = (x - self.min_x) / (self.max_x - self.min_x)
        norm_y = 1.0 - (y - self.min_y) / (self.max_y - self.min_y)

        cx, cy = self.width / 2.0, self.height / 2.0
        px = (norm_x * self.width - cx) * self.zoom + cx + self.pan_x
        py = (norm_y * self.height - cy) * self.zoom + cy + self.pan_y
        return int(px), int(py)

    def pixel_to_world(self, px, py):
        cx, cy = self.width / 2.0, self.height / 2.0
        norm_px = (px - self.pan_x - cx) / self.zoom + cx
        norm_py = (py - self.pan_y - cy) / self.zoom + cy

        x = self.min_x + (norm_px / self.width) * (self.max_x - self.min_x)
        y = self.min_y + (1.0 - norm_py / self.height) * (self.max_y - self.min_y)
        return x, y

    def center_on_world_pt(self, x, y):
        norm_x = (x - self.min_x) / (self.max_x - self.min_x)
        norm_y = 1.0 - (y - self.min_y) / (self.max_y - self.min_y)
        cx, cy = self.width / 2.0, self.height / 2.0
        self.pan_x = -(norm_x * self.width - cx) * self.zoom
        self.pan_y = -(norm_y * self.height - cy) * self.zoom

def main():
    selected_town = select_town()
    print(f"[Teleop] Connecting to CARLA server...")
    client = carla.Client(HOST, PORT_CARLA)
    client.set_timeout(20.0)

    print(f"[Teleop] Loading map: {selected_town}...")
    world = client.load_world(selected_town)
    carla_map = world.get_map()

    # Clear existing vehicle actors
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()

    # Spawn ego vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    vehicle_bp.set_attribute('role_name', 'hero')

    spawn_points = carla_map.get_spawn_points()
    spawn_transform = spawn_points[0]
    ego_vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
    print(f"[Teleop] Vehicle spawned at: {spawn_transform.location}")

    # Set CARLA Spectator Camera behind vehicle
    spectator = world.get_spectator()
    cam_transform = carla.Transform(
        spawn_transform.location + carla.Location(
            x=-8.0 * math.cos(math.radians(spawn_transform.rotation.yaw)),
            y=-8.0 * math.sin(math.radians(spawn_transform.rotation.yaw)),
            z=5.0
        ),
        carla.Rotation(pitch=-20.0, yaw=spawn_transform.rotation.yaw, roll=0.0)
    )
    spectator.set_transform(cam_transform)

    start_loc = spawn_transform.location
    start_geo = carla_map.transform_to_geolocation(start_loc)

    # Extract road topology
    topology = carla_map.get_topology()
    all_points = []
    for segment in topology:
        all_points.append([segment[0].transform.location.x, segment[0].transform.location.y])
        all_points.append([segment[1].transform.location.x, segment[1].transform.location.y])
    all_pts = np.array(all_points)

    padding = 30.0
    bounds = (
        all_pts[:, 0].min() - padding,
        all_pts[:, 0].max() + padding,
        all_pts[:, 1].min() - padding,
        all_pts[:, 1].max() + padding
    )

    pygame.init()
    pygame.font.init()
    font = pygame.font.SysFont('Consolas', 14)
    font_bold = pygame.font.SysFont('Consolas', 15, bold=True)
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption(f"CARLA Live Teleop & GPS Visualizer - {selected_town}")
    clock = pygame.time.Clock()

    map_view = InteractiveMap(bounds, WINDOW_WIDTH, WINDOW_HEIGHT)
    goal_world_coords = None
    planned_path = []
    tracking_active = False
    status_msg = "SELECT DESTINATION (Left Click)"

    print("\n--- Teleop & GPS Visualizer Controls ---")
    print("  * Left Click        : Select Target Destination")
    print("  * Right Click + Drag: Pan/Move Map")
    print("  * Mouse Wheel       : Zoom In / Zoom Out")
    print("  * ENTER             : Send Mission to RRT* Planner")
    print("  * 'C' Key           : Center Camera View on Vehicle\n")

    running = True
    while running:
        screen.fill((20, 20, 20))

        # Query live vehicle status from CARLA
        curr_transform = ego_vehicle.get_transform()
        curr_loc = curr_transform.location
        curr_yaw = curr_transform.rotation.yaw
        curr_vel = ego_vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(curr_vel.x**2 + curr_vel.y**2 + curr_vel.z**2)
        curr_geo = carla_map.transform_to_geolocation(curr_loc)

        # 1. Render Map Road Topology
        for segment in topology:
            p1 = map_view.world_to_pixel(segment[0].transform.location.x, segment[0].transform.location.y)
            p2 = map_view.world_to_pixel(segment[1].transform.location.x, segment[1].transform.location.y)
            pygame.draw.line(screen, (70, 70, 70), p1, p2, 2)

        # 2. Render Full Planned RRT* Path (Cyan Line)
        if len(planned_path) > 1:
            pixel_path = [map_view.world_to_pixel(pt[0], pt[1]) for pt in planned_path]
            pygame.draw.lines(screen, (0, 220, 255), False, pixel_path, 3)

        # 3. Render Initial Start Position (Green Marker)
        start_px = map_view.world_to_pixel(start_loc.x, start_loc.y)
        pygame.draw.circle(screen, (0, 255, 0), start_px, 6)

        # 4. Render Target Goal Position (Red Target)
        if goal_world_coords:
            goal_px = map_view.world_to_pixel(goal_world_coords[0], goal_world_coords[1])
            pygame.draw.circle(screen, (255, 50, 50), goal_px, 8)
            pygame.draw.circle(screen, (255, 255, 255), goal_px, 11, 2)

        # 5. Render Live Vehicle Position & Orientation Vector (Yellow Dot + Arrow)
        veh_px = map_view.world_to_pixel(curr_loc.x, curr_loc.y)
        pygame.draw.circle(screen, (255, 235, 0), veh_px, 9)
        pygame.draw.circle(screen, (0, 0, 0), veh_px, 10, 2)

        # Vehicle Direction Arrow
        dir_x = veh_px[0] + int(20 * math.cos(math.radians(curr_yaw)))
        dir_y = veh_px[1] + int(20 * math.sin(math.radians(curr_yaw)))
        pygame.draw.line(screen, (255, 255, 255), veh_px, (dir_x, dir_y), 3)

        # Crosshair cursor lines
        mx, my = pygame.mouse.get_pos()
        hx, hy = map_view.pixel_to_world(mx, my)
        hover_geo = carla_map.transform_to_geolocation(carla.Location(x=hx, y=hy, z=start_loc.z))

        # 6. On-Screen HUD Overlay
        dist_to_goal = math.hypot(goal_world_coords[0] - curr_loc.x, goal_world_coords[1] - curr_loc.y) if goal_world_coords else 0.0

        hud_lines = [
            ("STATUS       : " + status_msg, (0, 255, 200)),
            (f"TOWN / ZOOM  : {selected_town} | {map_view.zoom:.2f}x", (220, 220, 220)),
            (f"VEHICLE SPEED: {speed_kmh:.1f} km/h", (255, 235, 0)),
            (f"LIVE GPS     : Lat: {curr_geo.latitude:.6f}, Lon: {curr_geo.longitude:.6f}", (255, 255, 255)),
            (f"DIST TO GOAL : {dist_to_goal:.1f} meters" if goal_world_coords else "DIST TO GOAL : N/A", (255, 120, 120)),
            (f"CURSOR GPS   : Lat: {hover_geo.latitude:.6f}, Lon: {hover_geo.longitude:.6f}", (150, 150, 150))
        ]

        # Draw HUD Box Background
        pygame.draw.rect(screen, (10, 10, 10), (5, 5, 480, 145))
        pygame.draw.rect(screen, (80, 80, 80), (5, 5, 480, 145), 1)

        for idx, (text, color) in enumerate(hud_lines):
            lbl = font_bold.render(text, True, color) if idx == 0 else font.render(text, True, color)
            screen.blit(lbl, (12, 10 + idx * 21))

        # Pygame Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:  # Zoom In
                    map_view.zoom *= 1.15
                elif event.button == 5:  # Zoom Out
                    map_view.zoom = max(0.2, map_view.zoom / 1.15)
                elif event.button == 3:  # Right Click Drag Start
                    map_view.dragging = True
                    map_view.drag_start = event.pos
                elif event.button == 1 and not tracking_active:  # Select Goal
                    gx, gy = map_view.pixel_to_world(event.pos[0], event.pos[1])
                    goal_world_coords = (gx, gy, start_loc.z)
                    status_msg = "GOAL SET -> Press ENTER to Plan & Stream"
                    print(f"[Teleop] Destination set: World({gx:.2f}, {gy:.2f})")

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    map_view.dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if map_view.dragging:
                    dx = event.pos[0] - map_view.drag_start[0]
                    dy = event.pos[1] - map_view.drag_start[1]
                    map_view.pan_x += dx
                    map_view.pan_y += dy
                    map_view.drag_start = event.pos

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    # Center map on vehicle
                    map_view.center_on_world_pt(curr_loc.x, curr_loc.y)
                    print("[Teleop] Centered map view on vehicle.")

                elif event.key == pygame.K_RETURN and goal_world_coords is not None and not tracking_active:
                    goal_geo = carla_map.transform_to_geolocation(carla.Location(x=goal_world_coords[0], y=goal_world_coords[1], z=start_loc.z))
                    payload = {
                        "start": [start_loc.x, start_loc.y, start_loc.z],
                        "goal": list(goal_world_coords),
                        "start_gps": [start_geo.latitude, start_geo.longitude, start_geo.altitude],
                        "goal_gps": [goal_geo.latitude, goal_geo.longitude, start_geo.altitude],
                        "town": selected_town
                    }
                    
                    status_msg = "CONNECTING TO PLANNER..."
                    pygame.display.flip()

                    try:
                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.connect((HOST, PORT_PLANNER))
                        sock.sendall(json.dumps(payload).encode('utf-8'))
                        
                        # Receive planned trajectory back from planner
                        raw_resp = sock.recv(2097152)  # 2MB buffer
                        resp = json.loads(raw_resp.decode('utf-8'))
                        sock.close()

                        planned_path = resp.get("path", [])
                        tracking_active = True
                        status_msg = f"LIVE TRACKING ACTIVE ({len(planned_path)} Waypoints)"
                        print(f"[Teleop] Mission sent! Received path with {len(planned_path)} waypoints. Teleop remaining open for live tracking...")

                    except Exception as e:
                        status_msg = "ERROR: Start `RRTstar_mission_planner.py` first!"
                        print(f"[Teleop] Connection failed: {e}")

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == '__main__':
    main()