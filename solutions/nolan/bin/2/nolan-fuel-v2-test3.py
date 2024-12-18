from collections import defaultdict
import json
import heapq

DATASET_FILE = 'datasets/2_pacman.json'
predective_depth=10

def build_graph(roads):
    """
    Build a graph as an adjacency list.
    """
    graph = defaultdict(list)
    total_edges = 0
    total_length = 0
    
    for road in roads:
        start, end, is_one_way, length = road['intersectionId1'], road['intersectionId2'], road['isOneWay'], road['length']
        graph[start].append((end, length))
        total_edges += 1
        total_length += length
        if not is_one_way:
            graph[end].append((start, length))
    
    return graph, total_edges, total_length

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

def find_best_station(graph, days):
    """
    Find the best position for the charging station.
    """
    best_station = None
    max_reachable_roads = 0

    for node, neighbors in graph.items():  # Correction ici
        if len(neighbors) >= days + 1:  # Vérifie si le nœud a au moins days + 1 routes adjacentes
            reachable = reachable_roads(graph, node)
            if reachable > max_reachable_roads:
                max_reachable_roads = reachable
                best_station = node

    return best_station

def simulate_robot(graph, start_node, battery_capacity, num_days):
    """
    Simulate the robot's journey to maximize unique distance covered and return the full path.
    """
    visited_edges = set()  # Track unique edges visited
    edge_frequencies = defaultdict(int)  # Track how often each edge is used
    path = [start_node]  # Full path of nodes
    total_distance = 0
    current_node = start_node
    current_battery = battery_capacity
    day = 1

    def shortest_path_to_station(current_node):
        """
        Compute the shortest path from current_node to the station using Dijkstra's algorithm.
        """
        station = start_node
        pq = [(0, current_node)]  # (distance, node)
        distances = {node: float('inf') for node in graph}
        distances[current_node] = 0
        while pq:
            dist, node = heapq.heappop(pq)
            if node == station:
                return dist
            for neighbor, length in graph[node]:
                new_dist = dist + length
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    heapq.heappush(pq, (new_dist, neighbor))
        return float('inf')  # No path to the station

    def predict_next_moves(node, battery, depth=predective_depth):
        """
        Predict the best sequence of moves up to `depth` steps ahead, ensuring battery sufficiency.
        """
        if depth == 0 or battery <= 0:
            return 0

        max_distance = 0
        for neighbor, length in graph[node]:
            edge = (min(node, neighbor), max(node, neighbor))
            if battery < length:
                continue

            # Avoid unnecessary returns to the station
            if neighbor == start_node and battery > battery_capacity // 3:
                continue

            # Ensure the robot can return to the station if needed
            return_cost = shortest_path_to_station(neighbor)
            if return_cost > battery - length:
                continue

            # Temporarily mark edge as visited
            visited_edges.add(edge)
            distance = length + predict_next_moves(neighbor, battery - length, depth - 1)
            visited_edges.discard(edge)

            max_distance = max(max_distance, distance)

        return max_distance

    while day <= num_days:
        print(f"Day {day}: Starting at node {current_node} with full battery.")
        day_distance = 0

        while current_battery > 0:
            best_edge = None
            best_neighbor = None
            best_score = float('-inf')

            # Select the best edge
            single_neighbor = [
                (neighbor, length) for neighbor, length in graph[current_node]
                if (min(current_node, neighbor), max(current_node, neighbor)) not in visited_edges
            ]

            if len(single_neighbor) == 1:
                # Move directly to the single neighbor
                best_neighbor, edge_length = single_neighbor[0]
                best_edge = (min(current_node, best_neighbor), max(current_node, best_neighbor))
                print(f"Only one neighbor at node {current_node}. Moving directly to {best_neighbor}.")
            else:
                for neighbor, length in graph[current_node]:
                    if len(path) >= 2 and neighbor == path[-2]:
                        continue
                    edge = (min(current_node, neighbor), max(current_node, neighbor))

                    if current_battery >= length:
                        predicted_score = length + predict_next_moves(neighbor, current_battery - length)
                        # Penalize edges based on frequency of use
                        edge_penalty = edge_frequencies[edge] * 10
                        total_score = predicted_score - edge_penalty

                        if total_score > best_score:
                            best_score = total_score
                            best_edge = edge
                            best_neighbor = neighbor

            if best_neighbor is None:
                print("No more valid moves available.")
                break

            # Move to the best neighbor
            path.append(best_neighbor)
            edge_length = next(length for n, length in graph[current_node] if n == best_neighbor)
            current_battery -= edge_length

            # Update visited edges and frequency
            if best_edge not in visited_edges:
                visited_edges.add(best_edge)
                total_distance += edge_length
                day_distance += edge_length
            edge_frequencies[best_edge] += 1

            current_node = best_neighbor
            print(f"Moved to node {current_node}, remaining battery: {current_battery}")

            # Check if the robot returns to the station
            if best_neighbor == start_node:
                current_battery = 0  # Force recharge

        print(f"Day {day}: Covered {day_distance} units of unique distance.")
        day += 1  # Increment day at the end of each loop
        current_battery = battery_capacity  # Recharge battery for the new day

    print(f"Total unique distance covered: {total_distance}")
    return path


if __name__ == "__main__":
    with open(DATASET_FILE) as fi:
        dataset = fi.read()
    data = json.loads(dataset)

    graph, total_edges, total_length = build_graph(data["roads"])
    print("Graph constructed with", len(graph), "nodes")
    print("Total number of edges:", total_edges)

    battery_capacity = data["batteryCapacity"]
    recharge = data["numDays"] - 1
    print("Battery capacity:", battery_capacity)
    print("Number of recharges:", recharge)

    start_node = find_best_station(graph, data["numDays"])
    print("Best station:", start_node)

    robot_path = simulate_robot(graph, start_node, battery_capacity, data["numDays"])
    print(f"Robot's full path: {robot_path}")
