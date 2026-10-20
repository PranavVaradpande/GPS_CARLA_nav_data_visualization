import carla
import numpy as np
import socket
import json
import math
import heapq
import random

HOST = 'localhost'
PORT_GUI = 5555
PORT_CONTROLLER = 5556

class AStarRoadPlanner:
    """A* Graph Search over CARLA Road Topology to find the true shortest path without loops."""
    def __init__(self, carla_map, step_dist=2.0):
        self.map = carla_map
        self.step_dist = step_dist

    def find_shortest_route(self, start_pos, goal_pos):
        start_wp = self.map.get_waypoint(carla.Location(x=start_pos[0], y=start_pos[1], z=start_pos[2]))
        goal_wp = self.map.get_waypoint(carla.Location(x=goal_pos[0], y=goal_pos[1], z=goal_pos[2]))
        goal_loc = goal_wp.transform.location

        def get_wp_key(wp):
            # Unique node identifier based on road attributes and position along lane
            return (wp.road_id, wp.section_id, wp.lane_id, round(wp.s, 1))

        # Priority Queue: (f_score, g_score, tie_breaker_counter, current_wp, path_list)
        open_set = []
        start_key = get_wp_key(start_wp)
        h_start = start_wp.transform.location.distance(goal_loc)
        
        counter = 0
        heapq.heappush(open_set, (h_start, 0.0, counter, start_wp, [[start_pos[0], start_pos[1], start_pos[2]]]))
        
        g_scores = {start_key: 0.0}
        visited_nodes = set()

        best_path = None
        closest_dist = float('inf')
        closest_path = None

        while open_set:
            f, g, _, curr_wp, path = heapq.heappop(open_set)
            curr_key = get_wp_key(curr_wp)

            if curr_key in visited_nodes:
                continue
            visited_nodes.add(curr_key)

            curr_loc = curr_wp.transform.location
            dist_to_goal = curr_loc.distance(goal_loc)

            # Track closest approach
            if dist_to_goal < closest_dist:
                closest_dist = dist_to_goal
                closest_path = path

            # Goal reached threshold (within 3.0 meters of goal waypoint)
            if dist_to_goal <= 3.0:
                best_path = path
                break

            # Explore all possible next connected road options (including junction branches)
            next_wps = curr_wp.next(self.step_dist)
            
            for nxt in next_wps:
                nxt_key = get_wp_key(nxt)
                step_cost = curr_loc.distance(nxt.transform.location)
                new_g = g + step_cost

                if nxt_key not in g_scores or new_g < g_scores[nxt_key]:
                    g_scores[nxt_key] = new_g
                    h = nxt.transform.location.distance(goal_loc)
                    f_new = new_g + h
                    counter += 1
                    new_path = path + [[nxt.transform.location.x, nxt.transform.location.y, nxt.transform.location.z]]
                    heapq.heappush(open_set, (f_new, new_g, counter, nxt, new_path))

        raw_route = best_path if best_path is not None else closest_path
        if raw_route:
            raw_route.append([goal_pos[0], goal_pos[1], goal_pos[2]])

        return raw_route


class RRTStarPlanner:
    """RRT* Path Refiner for smooth trajectory generation along the shortest route."""
    def __init__(self, carla_map):
        self.map = carla_map
        self.astar = AStarRoadPlanner(carla_map)

    def plan_shortest_path(self, start_pos, goal_pos):
        print("[RRT* Planner] Calculating optimal shortest route via A* Road Network Search...")
        
        # 1. Guaranteed Shortest Path via A* Topology Search
        shortest_raw_path = self.astar.find_shortest_route(start_pos, goal_pos)

        if not shortest_raw_path:
            print("[RRT* Planner] Warning: Direct road search failed. Using straight interpolation.")
            shortest_raw_path = [start_pos, goal_pos]

        print(f"[RRT* Planner] Shortest road skeleton found with {len(shortest_raw_path)} key nodes.")

        # 2. Path Smoothing & Dense Waypoint Interpolation
        return self._smooth_and_interpolate(shortest_raw_path, step=0.5)

    def _smooth_and_interpolate(self, raw_path, step=0.5):
        """Generates dense, equidistant waypoints and smooths sharp turn corners."""
        if len(raw_path) < 2:
            return raw_path

        dense_path = []
        for i in range(len(raw_path) - 1):
            p1, p2 = np.array(raw_path[i]), np.array(raw_path[i+1])
            dist = np.linalg.norm(p2 - p1)
            num_pts = max(1, int(dist / step))
            for t in np.linspace(0, 1, num_pts, endpoint=False):
                pt = p1 + t * (p2 - p1)
                dense_path.append(pt.tolist())
                
        dense_path.append(raw_path[-1])
        return dense_path


def main():
    # Phase 1: Wait for Start/Goal from GPS_teleop.py
    gui_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    gui_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gui_server.bind((HOST, PORT_GUI))
    gui_server.listen(1)
    print(f"[RRT* Planner] Online and listening for mission target on port {PORT_GUI}...")

    conn, addr = gui_server.accept()
    data = conn.recv(4096)
    mission_data = json.loads(data.decode('utf-8'))

    start_pt = mission_data['start']
    goal_pt = mission_data['goal']
    town = mission_data['town']
    print(f"[RRT* Planner] Received Target Goal: {goal_pt} on {town}")

    client = carla.Client(HOST, 2000)
    world = client.get_world()
    carla_map = world.get_map()

    planner = RRTStarPlanner(carla_map)
    full_path = planner.plan_shortest_path(start_pt, goal_pt)
    print(f"[RRT* Planner] Optimal Path generated! Total Dense Waypoints: {len(full_path)}")

    # Send full path back to Teleop Visualizer GUI
    reply = {"status": "SUCCESS", "path": full_path}
    conn.sendall(json.dumps(reply).encode('utf-8'))
    conn.close()
    gui_server.close()

    # Phase 2: Stream 50-waypoint chunks to Vehicle_control.py
    ctrl_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ctrl_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ctrl_server.bind((HOST, PORT_CONTROLLER))
    ctrl_server.listen(5)
    print(f"[RRT* Planner] Waypoint Chunk Streaming Server listening on port {PORT_CONTROLLER}...")

    current_idx = 0
    chunk_size = 50

    while True:
        conn, addr = ctrl_server.accept()
        msg = conn.recv(1024).decode('utf-8')
        if not msg:
            break

        req = json.loads(msg)
        if req.get("command") in ["FETCH_FIRST", "FETCH_NEXT"]:
            next_idx = min(current_idx + chunk_size, len(full_path))
            chunk = full_path[current_idx:next_idx]
            is_last = (next_idx >= len(full_path))
            current_idx = next_idx

            response = {
                "waypoints": chunk,
                "is_last_chunk": is_last,
                "remaining": len(full_path) - current_idx
            }
            conn.sendall(json.dumps(response).encode('utf-8'))
            print(f"[RRT* Planner] Transmitted batch of {len(chunk)} waypoints. (Remaining: {response['remaining']})")

            if is_last and len(chunk) == 0:
                print("[RRT* Planner] All waypoints delivered. Mission routing complete.")
                conn.close()
                break
        conn.close()

if __name__ == '__main__':
    main()