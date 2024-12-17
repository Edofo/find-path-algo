from collections import defaultdict

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

def traverse_all_edges(graph, start, total_edges, total_length, battery_capacity, recharge):
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

        if visited_count == total_edges or current_fuel <= 0:
            break

        for i, (next_node, length) in enumerate(graph[current_node]):
            edge = (min(current_node, next_node), max(current_node, next_node))
            print("Current node:", current_node, "Next node:", next_node, "Edge:", edge)

            edges = graph[current_node]
            is_last_edge = i == len(edges) - 1

            # check if the edge is the last not visited edge from the current node
            unvisited_edges = [
                (min(current_node, neighbor), max(current_node, neighbor)) 
                for neighbor, _ in edges 
                if (min(current_node, neighbor), max(current_node, neighbor)) not in visited_edges
            ]

            is_last_unvisited_edge = len(unvisited_edges) == 1 and edge == unvisited_edges[0]

            neighbors = graph[next_node]
            neighbors = [neighbor for neighbor in neighbors if neighbor[0] != current_node]
            all_visited = all((min(next_node, neighbor[0]), max(next_node, neighbor[0])) in visited_edges for neighbor in neighbors)
            if all_visited and not is_last_edge and not is_last_unvisited_edge:
                continue

            # print("Ttee", total_length, battery_capacity, (battery_capacity - current_fuel), current_fuel)
            if next_node == start and visited_count + (battery_capacity / 10) < total_edges:
                print("Doest not have enough fuel to travel all edges")
                continue

            if edge not in visited_edges and current_fuel - length >= 0 or is_last_edge or is_last_unvisited_edge:
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
    data = {
        "comment": "Simple layout with 7 nodes in a hexagonal pattern",
        "batteryCapacity": 70,
        "numDays": 2,
        "intersections": [
            {"id": 0, "lat": 5.0, "lng": -8.66},
            {"id": 1, "lat": 10.0, "lng": 0.0},
            {"id": 2, "lat": 5.0, "lng": 8.66},
            {"id": 3, "lat": -5.0, "lng": 8.66},
            {"id": 4, "lat": -10.0, "lng": 0.0},
            {"id": 5, "lat": -5.0, "lng": -8.66},
            {"id": 6, "lat": 0.0, "lng": 0.0}
        ],
        "roads": [
            {"intersectionId1": 0, "intersectionId2": 1, "isOneWay": False, "length": 10},
            {"intersectionId1": 0, "intersectionId2": 6, "isOneWay": True, "length": 10},
            {"intersectionId1": 0, "intersectionId2": 5, "isOneWay": False, "length": 10},
            {"intersectionId1": 1, "intersectionId2": 6, "isOneWay": False, "length": 10},
            {"intersectionId1": 1, "intersectionId2": 2, "isOneWay": False, "length": 10},
            {"intersectionId1": 6, "intersectionId2": 2, "isOneWay": True, "length": 10},
            {"intersectionId1": 2, "intersectionId2": 3, "isOneWay": False, "length": 10},
            {"intersectionId1": 3, "intersectionId2": 6, "isOneWay": False, "length": 10},
            {"intersectionId1": 3, "intersectionId2": 4, "isOneWay": False, "length": 10},
            {"intersectionId1": 4, "intersectionId2": 6, "isOneWay": False, "length": 10},
            {"intersectionId1": 4, "intersectionId2": 5, "isOneWay": False, "length": 10},
            {"intersectionId1": 5, "intersectionId2": 6, "isOneWay": False, "length": 10}
        ]
    }

    graph, total_edges, total_length = build_graph(data["roads"])
    print("Graph constructed:", dict(graph))
    print("Total number of edges:", total_edges)

    battery_capacity = data["batteryCapacity"]
    recharge = data["numDays"] - 1

    start_node = get_start_node(graph)
    path = traverse_all_edges(graph, start_node, total_edges, total_length, battery_capacity, recharge)
    print("Traveling all edges:", path)
