from collections import defaultdict, deque
import json

DATASET_FILE = 'datasets/2_pacman.json'

def build_graph(intersections, roads):
    graph = {}
    for intersection in intersections:
        graph[intersection['id']] = []
    for road in roads:
        graph[road['intersectionId1']].append((road['intersectionId2'], road['isOneWay']))
        if not road['isOneWay']:
            graph[road['intersectionId2']].append((road['intersectionId1'], False))
    return graph

def select_best_station(graph, battery_capacity):
    max_reachable_roads = 0
    best_station = None
    
    for station in graph.keys():
        visited, roads_count = bfs_count_roads(graph, station, battery_capacity)
        if roads_count > max_reachable_roads:
            max_reachable_roads = roads_count
            best_station = station
    return best_station

def plan_routes(graph, station, battery_capacity, num_days):
    visited_edges = set()
    daily_routes = []
    
    for day in range(num_days):
        print(f"Jour {day + 1}")
        visited, route = dfs_maximize_roads(graph, station, battery_capacity, visited_edges)
        visited_edges.update(route)
        daily_routes.append(route)
    
    return daily_routes

# BFS pour compter les rues atteignables à partir d'une station
def bfs_count_roads(graph, start, battery_capacity):
    queue = deque([(start, 0)])
    visited_nodes = set()
    visited_edges = set()
    
    while queue:
        node, distance = queue.popleft()
        if distance >= battery_capacity or node in visited_nodes:
            continue
        visited_nodes.add(node)
        
        for neighbor, is_one_way in graph[node]:
            if (node, neighbor) not in visited_edges:
                visited_edges.add((node, neighbor))
                if not is_one_way:
                    visited_edges.add((neighbor, node))
                queue.append((neighbor, distance + 1))
    
    return visited_nodes, len(visited_edges)

# DFS modifié pour maximiser les rues parcourues
def dfs_maximize_roads(graph, start, battery_capacity, visited_edges):
    stack = [(start, 0, [])]
    max_route = []
    
    while stack:
        node, distance, path = stack.pop()
        print(f"Node: {node}, Distance: {distance}")
        if distance >= battery_capacity:
            continue
        for neighbor, is_one_way in graph[node]:
            if (node, neighbor) not in visited_edges:
                new_path = path + [(node, neighbor)]
                stack.append((neighbor, distance + 1, new_path))
                if len(new_path) > len(max_route):
                    max_route = new_path
    
    return set(edge for edge in max_route), max_route

# Programme principal
def main(data):
    graph = build_graph(data['intersections'], data['roads'])
    battery_capacity = data['batteryCapacity']
    num_days = data['numDays']
    
    # Étape 1 : Trouver la meilleure station
    best_station = select_best_station(graph, battery_capacity)
    print(f"Meilleure station : {best_station}") # 190
    
    # Étape 2 : Générer les itinéraires
    routes = plan_routes(graph, best_station, battery_capacity, num_days)
    print(f"Itinéraires : {routes}")
    
    return best_station, routes

with open(DATASET_FILE) as fi:
    dataset = fi.read()
data = json.loads(dataset)

station, routes = main(data)
print(f"Station de recharge : {station}")
for day, route in enumerate(routes, 1):
    print(f"Jour {day} : {route}")