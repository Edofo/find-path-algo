from collections import defaultdict

DATASET_FILE = 'datasets/2_pacman.json'

import json

def build_graph(roads):
    """Construct the graph with directed and undirected edges."""
    graph = defaultdict(list)
    total_edges = 0
    total_length = 0

    for road in roads:
        u, v, is_one_way, length = road["intersectionId1"], road["intersectionId2"], road["isOneWay"], road["length"]
        graph[u].append((v, length))
        total_edges += 1
        total_length += length
        if not is_one_way:
            graph[v].append((u, length))

    return graph, total_edges, total_length

def traverse_all_edges(graph, start, total_edges, battery_capacity, recharge):
    """Travels all edges with the least possible cost"""
    visited_edges = set()
    stack = [start]
    path = []
    visited_count = 0 
    current_fuel = battery_capacity
    recharge_count = 0

    while stack:
        current_node = stack[-1]
        print("Current fuel:", current_fuel)

        if visited_count == total_edges or current_fuel <= 0 or recharge_count > recharge:
            break

        for i, (next_node, length) in enumerate(graph[current_node]):
            edge = (min(current_node, next_node), max(current_node, next_node))
            print("Current node:", current_node, "Next node:", next_node, "Edge:", edge)
            
            if len(path) >= 2 and path[len(path) - 1] == next_node:
                print("Backtracking to node:", next_node)
                continue

            if current_fuel - length <= 0 and next_node != start:
                print("Not enough fuel to travel to next node")
                continue

            if next_node == start and visited_count + (battery_capacity / 1) < total_edges:
                print("Doest not have enough fuel to travel all edges")
                continue

            if edge not in visited_edges:
                visited_edges.add(edge)
                visited_count += 1
            path.append(current_node)
            stack.append(next_node)
            current_fuel -= length
            if next_node == start:
                print("Recharging at node:", current_node)
                current_fuel = battery_capacity
                recharge_count += 1
            print("Visited edges:", visited_edges)
            break

        else:   
            print("Backtracking from node:", current_node)
            stack.pop()

    return path, visited_count

if __name__ == "__main__":
    with open(DATASET_FILE) as fi:
        dataset = fi.read()
    data = json.loads(dataset)
    graph, total_edges, total_length = build_graph(data["roads"])
    print("Graph constructed:", dict(graph))
    print("Total number of edges:", total_edges)

    battery_capacity = data["batteryCapacity"]
    recharge = data["numDays"] - 1
    print("Battery capacity:", battery_capacity)
    print("Recharge:", recharge)

    start_node = 0
    path, visited_count = traverse_all_edges(graph, start_node, total_edges, battery_capacity, recharge)
    print("Traveling path:", path)
    print("Visited edges:", visited_count, "Total edges:", total_edges)
