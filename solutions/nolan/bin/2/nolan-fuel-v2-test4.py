from collections import defaultdict
import json
from heapq import heappush, heappop

DATASET_FILE = 'datasets/2_pacman.json'
predective_depth=15

def build_graph(roads, intersections):
    """
    Build a graph as an adjacency list, and include latitude/longitude for each node.
    """
    graph = defaultdict(list)
    node_coords = {}  # Store latitude and longitude for each node
    total_edges = 0
    total_length = 0

    # Add intersection coordinates
    for intersection in intersections:
        node_coords[intersection["id"]] = (intersection["lat"], intersection["lng"])

    # Build adjacency list
    for road in roads:
        start, end, is_one_way, length = road['intersectionId1'], road['intersectionId2'], road['isOneWay'], road['length']
        graph[start].append((end, length))
        total_edges += 1
        total_length += length
        if not is_one_way:
            graph[end].append((start, length))
    
    return graph, node_coords, total_edges, total_length

def reachable_roads(graph, start):
    """
    Explore toutes les routes accessibles depuis un nœud donné (sans limite de batterie).
    """
    visited_nodes = set()
    visited_edges = set()
    stack = [start]

    while stack:
        node = stack.pop()
        if node in visited_nodes:
            continue
        visited_nodes.add(node)

        for neighbor in graph[node]:
            edge = (min(node, neighbor[0]), max(node, neighbor[0]))  # Normalisation des arêtes
            if edge not in visited_edges:
                visited_edges.add(edge)
                stack.append(neighbor[0])

    return len(visited_edges)

def find_best_station(graph, node_coords, weight_reachable=10, weight_distance=1, weight_adjacent=5):
    """
    Find the best position for the charging station by combining geographic center,
    reachable roads, and adjacent routes, with adjustable weights for criteria.
    """
    # Calculate the center of the map
    total_lat = total_lng = 0
    num_nodes = len(node_coords)

    for lat, lng in node_coords.values():
        total_lat += lat
        total_lng += lng

    center_lat = total_lat / num_nodes
    center_lng = total_lng / num_nodes

    # Helper function: Calculate distance between two points
    def distance(lat1, lng1, lat2, lng2):
        return ((lat1 - lat2) ** 2 + (lng1 - lng2) ** 2) ** 0.5

    # Evaluate each node based on the combined criteria
    best_station = None
    best_score = float('-inf')
    scores = []  # For debugging purposes

    for node, (lat, lng) in node_coords.items():
        dist_to_center = distance(lat, lng, center_lat, center_lng)
        reachable = reachable_roads(graph, node)  # Count reachable roads
        adjacent_routes = len(graph[node])  # Count adjacent routes

        # Weighted score
        score = (weight_reachable * reachable) - (weight_distance * dist_to_center) + (weight_adjacent * adjacent_routes)
        scores.append((node, score, reachable, dist_to_center, adjacent_routes))

        if score > best_score:
            best_score = score
            best_station = node

    # Debug: Print top candidates for manual validation
    print("Top candidates:")
    for node, score, reachable, dist, adj in sorted(scores, key=lambda x: -x[1])[:10]:
        print(f"Node {node}: Score={score:.2f}, Reachable={reachable}, Distance={dist:.2f}, Adjacent={adj}")

    print(f"Geographic center is approximately at ({center_lat}, {center_lng})")
    return best_station

def simulate_robot(graph, start_node, battery_capacity, num_days):
    """
    Simulates the robot's journey to maximize the distance covered over a fixed number of days.

    Parameters:
        graph: dict - adjacency list of the graph
        start_node: int - the node where the station is located
        battery_capacity: int - the fuel capacity per day
        num_days: int - total number of days available

    Returns:
        list - the sequence of nodes visited by the robot
    """
    path = [start_node]  # Full path of the robot, starting with the station
    visited_edges = set()  # Set to track unique edges visited
    current_node = start_node
    current_battery = battery_capacity
    day = 1

    def shortest_path(src, target):
        """
        Compute the shortest path between src and target and return the path and distance.
        """
        pq = [(0, src, [])]  # Priority queue (distance, node, path)
        distances = {node: float('inf') for node in graph}
        distances[src] = 0

        while pq:
            dist, node, current_path = heappop(pq)
            if node == target:
                return current_path + [target], dist
            for neighbor, length in graph[node]:
                new_dist = dist + length
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    heappush(pq, (new_dist, neighbor, current_path + [neighbor]))
        return [], float('inf')  # No path to target

    while day <= num_days:
        print(f"Day {day}: Starting at node {current_node} with battery {current_battery}.")
        day_path = []  # Path for the current day
        while current_battery > 0:
            # Select the best move
            best_neighbor = None
            best_edge = None
            best_score = float('-inf')
            best_length = 0

            for neighbor, length in graph[current_node]:
                edge = (min(current_node, neighbor), max(current_node, neighbor))
                if edge in visited_edges or current_battery < length:
                    continue

                # Ensure the robot can return to the station
                next_path, return_cost = shortest_path(neighbor, start_node)
                if return_cost > current_battery - length:
                    continue

                # Score prioritizes unvisited edges and lower return costs
                score = length - return_cost
                if score > best_score:
                    best_score = score
                    best_neighbor = neighbor
                    best_edge = edge
                    best_length = length

            if not best_neighbor:
                # No valid moves left, return to the station
                print("No more valid moves, returning to the station.")
                break

            # Move to the best neighbor
            next_path, _ = shortest_path(current_node, best_neighbor)
            path.extend(next_path[1:])  # Add the full path to the neighbor, avoiding duplicates
            day_path.extend(next_path[1:])
            visited_edges.add(best_edge)
            current_battery -= best_length
            current_node = best_neighbor
            print(f"Moved to node {current_node}, remaining battery: {current_battery}.")

        # End of the day: Return to the station manually
        if current_node != start_node:
            print("Returning to the station to recharge.")
            return_path, return_cost = shortest_path(current_node, start_node)
            if return_cost <= current_battery:
                path.extend(return_path[1:])  # Append the path back to the station
                day_path.extend(return_path[1:])
                current_node = start_node
                current_battery = 0  # Force recharge
            else:
                raise ValueError("Insufficient battery to return to the station.")

        # Avoid duplicating the station in the path
        if path[-1] != start_node:
            path.append(start_node)

        # Recharge and prepare for the next day
        current_battery = battery_capacity
        day += 1

    print(f"Simulation complete. Total nodes visited: {len(set(path))}")
    return path

if __name__ == "__main__":
    with open(DATASET_FILE) as fi:
        dataset = fi.read()
    data = json.loads(dataset)

    graph, node_coords, total_edges, total_length = build_graph(data["roads"], data["intersections"])
    print("Graph constructed with", len(graph), "nodes")
    print("Total number of edges:", total_edges)

    battery_capacity = data["batteryCapacity"]
    recharge = data["numDays"] - 1
    print("Battery capacity:", battery_capacity)
    print("Number of recharges:", recharge)

    start_node = find_best_station(graph, node_coords)
    print("Best station:", start_node)

    robot_path = simulate_robot(graph, start_node, battery_capacity, data["numDays"])
    print(f"Robot's full path: {robot_path}")
