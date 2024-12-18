from collections import defaultdict
import json
from heapq import heappush, heappop

DATASET_FILE = 'datasets/3_efrei.json'

def build_graph(roads, intersections):
    """
    Build a graph as an adjacency list, and include latitude/longitude for each node.
    """
    graph = defaultdict(list)
    node_coords = {} 
    total_edges = 0
    total_length = 0

    for intersection in intersections:
        node_coords[intersection["id"]] = (intersection["lat"], intersection["lng"])

    for road in roads:
        start, end, is_one_way, length = road['intersectionId1'], road['intersectionId2'], road['isOneWay'], road['length']
        graph[start].append((end, length))
        total_edges += 1
        total_length += length
        if not is_one_way:
            graph[end].append((start, length))
    
    return graph, node_coords, total_edges, total_length

def choose_station(graph):
    return max(graph.keys(), key=lambda node: len(graph[node]))

def dijkstra(graph, start, target=None):
    """
    Dijkstra's algorithm to find the shortest path from start to target.
    If target is None, calculate the shortest path to all nodes.
    """
    heap = [(0, start)]  # (distance, node)
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parents = {start: None}
    
    while heap:
        curr_dist, curr_node = heappop(heap)
        
        if target is not None and curr_node == target:
            break  # Stop early if we reached the target
        
        for neighbor, weight in graph[curr_node]:
            distance = curr_dist + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parents[neighbor] = curr_node
                heappush(heap, (distance, neighbor))
    
    if target is not None:
        # Reconstruct path from start to target
        path = []
        while target is not None:
            path.append(target)
            target = parents[target]
        return path[::-1], distances[path[0]]  # Return path and its distance
    return distances

def find_best_return_path(graph, current_node, battery_left, visited_edges, station):
    """
    Find the best return path to the station:
    - Maximize the number of unvisited edges covered during the return.
    - Prioritize paths with more unvisited edges and longer distances.
    """
    def score_path(path, distances):
        """
        Calculate the score of a path based on:
        - Number of unvisited edges.
        - Length of the path (favoring longer paths).
        """
        score = 0
        covered_edges = set()
        total_distance = distances[path[-1]]  # Total distance of the path

        for i in range(len(path) - 1):
            edge = (min(path[i], path[i + 1]), max(path[i], path[i + 1]))
            if edge not in visited_edges:
                score += 1  # Increase score for each unvisited edge
                covered_edges.add(edge)

        # Combine score and distance into a weighted score
        # Adjust weight dynamically: prioritize unvisited edges, but balance with distance
        weight_unvisited = 20  # Weight for unvisited edges
        weighted_score = score * weight_unvisited - total_distance
        return weighted_score, covered_edges

    # Dijkstra to find paths to the station
    distances = {node: float('inf') for node in graph}
    parents = {node: None for node in graph}
    distances[current_node] = 0
    heap = [(0, current_node)]  # Min-heap: (distance, node)

    best_path = None
    best_weighted_score = float('-inf')  # Initialize to negative infinity to find max
    best_covered_edges = set()

    while heap:
        curr_dist, curr_node = heappop(heap)

        if curr_node == station:
            # Reconstruct the path to the station
            path = []
            temp_node = station
            while temp_node is not None:
                path.append(temp_node)
                temp_node = parents[temp_node]
            path = path[::-1]  # Reverse to get the correct order

            # Score this path
            path_score, covered_edges = score_path(path, distances)
            if path_score > best_weighted_score:
                best_path = path
                best_weighted_score = path_score
                best_covered_edges = covered_edges

        # Explore neighbors
        for neighbor, length in graph[curr_node]:
            new_distance = distances[curr_node] + length
            if new_distance < distances[neighbor] and new_distance <= battery_left:
                distances[neighbor] = new_distance
                parents[neighbor] = curr_node
                heappush(heap, (new_distance, neighbor))

    # If no path is found, fallback to the closest reachable node
    if not best_path:
        print(f"No valid path to station from {current_node}. Trying fallback.")
        closest_node, closest_dist = None, float('inf')
        for node in distances:
            if distances[node] < closest_dist and distances[node] <= battery_left:
                closest_node, closest_dist = node, distances[node]
        if closest_node is not None:
            # Reconstruct the path to the closest node
            path = []
            temp_node = closest_node
            while temp_node is not None:
                path.append(temp_node)
                temp_node = parents[temp_node]
            path = path[::-1]
            return path, closest_dist, set()  # No covered edges in fallback
        else:
            return [], 0, set()  # No valid path at all

    return best_path, distances[best_path[-1]], best_covered_edges

def calculate_edge_priority(graph, current_node, neighbor, length, visited_edges):
    """
    Calculate the priority of a route based on its length, centrality, and whether it has been visited.
    """
    edge = (min(current_node, neighbor), max(current_node, neighbor))
    centrality = len(graph[neighbor])  # Number of connections for the neighbor
    visited_penalty = 100 if edge in visited_edges else 0  # Penalize visited edges
    return length + centrality - visited_penalty

def maximize_battery_usage(graph, current_node, battery_left, visited_edges, last_day=False):
    """
    Attempt to use all remaining battery to explore more routes.
    Prioritize routes based on strategic scoring.
    """
    additional_path = []
    total_additional_distance = 0
    recent_nodes = []  # List to track recently visited nodes for loop detection
    max_recent_nodes = 5  # Limit to detect small loops

    while battery_left > 0:
        # Get all valid edges, including visited ones if last_day=True
        heap = []
        for neighbor, length in graph[current_node]:
            edge = (min(current_node, neighbor), max(current_node, neighbor))
            if last_day and battery_left >= length:
                # Allow revisiting edges on the last day if no unvisited routes are available
                priority = -calculate_edge_priority(graph, current_node, neighbor, length, visited_edges)
                # Penalize revisits if the node is in the recent history
                if neighbor in recent_nodes:
                    priority += 100  # High penalty for nodes in recent history
                heappush(heap, (priority, neighbor, length))
            elif edge not in visited_edges and battery_left >= length:
                # Prioritize unvisited edges
                priority = -calculate_edge_priority(graph, current_node, neighbor, length, visited_edges)
                heappush(heap, (priority, neighbor, length))

        if not heap:
            # No more valid routes
            break

        # Take the best route
        _, next_node, length = heappop(heap)

        # Detect loops and prevent revisiting recent nodes
        if next_node in recent_nodes:
            print(f"Loop detected: {current_node} -> {next_node}. Forcing alternative.")
            continue  # Skip this route to break the loop

        # Update recent nodes list (FIFO)
        recent_nodes.append(current_node)
        if len(recent_nodes) > max_recent_nodes:
            recent_nodes.pop(0)

        # Add the route to the path
        additional_path.append(next_node)
        if (min(current_node, next_node), max(current_node, next_node)) not in visited_edges:
            visited_edges.add((min(current_node, next_node), max(current_node, next_node)))
        battery_left -= length
        total_additional_distance += length
        current_node = next_node

    return additional_path, total_additional_distance, battery_left

def daily_route(graph, start, battery_capacity, visited_edges, station, last_day=False):
    """
    Plan a single day's route for the robot.
    - Use all available battery to maximize the distance covered.
    - Delay returning to the station until no more exploration is possible.
    - On the last day, don't return to the station.
    """
    current_node = start
    battery_left = battery_capacity
    daily_path = []
    total_distance = 0

    while True:
        # Priority queue to select the best next route
        heap = []
        for neighbor, length in graph[current_node]:
            edge = (min(current_node, neighbor), max(current_node, neighbor))
            if battery_left >= length:  # Valid route if battery is sufficient
                if edge not in visited_edges:
                    # Prioritize unvisited edges with longest length and strategic connections
                    priority = -length * 3  # Prioritize longer routes
                    heappush(heap, (priority, neighbor, length))
                elif battery_left >= (2 * battery_capacity) // 3:  # Allow revisiting if battery is high
                    # Penalize revisited edges, but allow them if battery is high
                    priority = -length // 2  # Penalize revisits (length is less important)
                    heappush(heap, (priority, neighbor, length))

        if not heap:
            # No more valid routes to explore, decide whether to return or advance
            if not last_day:
                # Try to return to the station with remaining battery
                return_path, return_distance, covered_edges = find_best_return_path(graph, current_node, battery_left, visited_edges, station)
                print(f"Returning to the station with {battery_left} battery left. Distance: {return_distance}")
                if return_path and return_distance <= battery_left:
                    daily_path.extend(return_path[1:])  # Skip current node (already included)
                    total_distance += return_distance
                    battery_left -= return_distance
                    visited_edges.update(covered_edges)
                break

            else:
                # On the last day, maximize exploration without returning to the station
                extra_path, extra_distance, battery_left = maximize_battery_usage(graph, current_node, battery_left, visited_edges, last_day=True)
                daily_path.extend(extra_path)
                total_distance += extra_distance
                break

        # Select the longest valid route or the best fallback option
        _, next_node, length = heappop(heap)
        daily_path.append(next_node)
        visited_edges.add((min(current_node, next_node), max(current_node, next_node)))
        battery_left -= length
        total_distance += length
        current_node = next_node

    return daily_path, total_distance, battery_left, visited_edges

def multi_day_route(graph, station, num_days, battery_capacity):
    """
    Plan the robot's multi-day route starting from the station.
    """
    visited_edges = set()
    total_distance = 0
    itinerary = [station]

    for day in range(num_days):
        print(f"Planning day {day + 1}...")
        last_day = (day == num_days - 1)  # Check if it's the last day

        path, distance, battery_left, visited_edges = daily_route(graph, station, battery_capacity, visited_edges, station, last_day=last_day)
        total_distance += distance
        visited_edges = visited_edges.union((min(node, neighbor), max(node, neighbor)) for node, neighbor in zip(path, path[1:]))
        itinerary.extend(path)

        print(f"Day {day + 1} completed: Distance covered = {distance}, Total distance = {total_distance}, Battery left = {battery_left}")

    return itinerary, total_distance, visited_edges

def convert_to_json(station_id, path):
    """
    Convert the result to JSON format
    """
    itinerary = [station_id] 
    for edge in path:
        if edge != itinerary[-1]: 
            itinerary.append(edge)
    
    result = {
        "chargeStationId": station_id,
        "itinerary": itinerary
    }
    
    return json.dumps(result, indent=2)

if __name__ == "__main__":
    with open(DATASET_FILE) as fi:
        dataset = fi.read()
    data = json.loads(dataset)

    graph, node_coords, total_edges, total_length = build_graph(data['roads'], data['intersections'])
    print("Graph constructed with", len(graph), "nodes")
    print("Total number of edges:", total_edges)

    best_station = choose_station(graph)
    # best_station = 195
    print("Best station:", best_station)

    itinerary, total_distance, visited_edges = multi_day_route(graph, best_station, data['numDays'], data['batteryCapacity'])
    print("Itinerary:", itinerary)
    print("Total distance covered:", total_distance)
    print("Number of unique edges visited:", len(visited_edges))

    json_result = convert_to_json(best_station, itinerary)
    # create a file and write the json result to it
    with open('output/3_submission.json', 'w') as f:
        f.write(json_result)

