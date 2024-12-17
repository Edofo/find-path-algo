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

def get_start_node(graph):
    """Find the node with the least number of edges"""
    min_edges = float("inf")
    start_node = None

    for node, edges in graph.items():
        if len(edges) < min_edges:
            min_edges = len(edges)
            start_node = node

    return start_node

def check_neighbors_visited(graph, current_node, next_node, visited_edges):
    """Check if all neighbors of the current node are visited"""
    neighbors = graph[next_node]
    neighbors = [neighbor for neighbor in neighbors if neighbor[0] != current_node]
    all_visited = all((min(next_node, neighbor[0]), max(next_node, neighbor[0])) in visited_edges for neighbor in neighbors)
    print("All neighbors visited:", all_visited)
    return all_visited

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
        path.append(current_node)
        print("Current fuel:", current_fuel)

        if visited_count == total_edges or current_fuel <= 0 or recharge_count > recharge:
            break

        for i, (next_node, length) in enumerate(graph[current_node]):
            edge = (min(current_node, next_node), max(current_node, next_node))
            print("Current node:", current_node, "Next node:", next_node, "Edge:", edge)

            edges = graph[current_node]
            is_last_edge = i == len(edges) - 1

            unvisited_edges = [
                (min(current_node, neighbor), max(current_node, neighbor)) 
                for neighbor, _ in edges 
                if (min(current_node, neighbor), max(current_node, neighbor)) not in visited_edges
            ]

            is_last_unvisited_edge = len(unvisited_edges) == 1 and edge == unvisited_edges[0]

            if check_neighbors_visited(graph, current_node, next_node, visited_edges) and not is_last_edge and not is_last_unvisited_edge:
                continue

            count = 0
            if edge in visited_edges and is_last_edge:
                curr = next_node
                for neighbor, length in graph[current_node]:
                    if not check_neighbors_visited(graph, current_node, neighbor, visited_edges):
                        count += 1
                        curr = neighbor
                if count == 1:
                    next_node = curr
                    edge = (min(current_node, next_node), max(current_node, next_node))
                    print("Next node:", next_node)

            if next_node == start and visited_count + (battery_capacity / 1) < total_edges:
                print("Doest not have enough fuel to travel all edges")
                continue

            if edge not in visited_edges and current_fuel - length >= 0 or is_last_edge or is_last_unvisited_edge or count == 1:
                if edge not in visited_edges:
                    visited_edges.add(edge)
                    visited_count += 1
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

    return path

if __name__ == "__main__":
    with open(DATASET_FILE) as fi:
        dataset = fi.read()
    data = json.loads(dataset)
    graph, total_edges, total_length = build_graph(data["roads"])
    print("Graph constructed:", dict(graph))
    print("Total number of edges:", total_edges)

    # battery_capacity = data["batteryCapacity"]
    # recharge = data["numDays"] - 1

    # start_node = get_start_node(graph)
    # path = traverse_all_edges(graph, start_node, total_edges, battery_capacity, recharge)
    # print("Traveling all edges:", path)
