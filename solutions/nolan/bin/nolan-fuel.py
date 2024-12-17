from collections import defaultdict

def build_graph(roads):
    """Construct the graph with directed and undirected edges."""
    graph = defaultdict(list)
    total_edges = 0

    for road in roads:
        u, v, is_one_way, length = road["intersectionId1"], road["intersectionId2"], road["isOneWay"], road["length"]
        graph[u].append((v, is_one_way, length))
        total_edges += 1
        if not is_one_way:
            graph[v].append((u, is_one_way, length))

    return graph, total_edges

def get_start_node(graph):
    """Find the node with the least number of edges"""
    min_edges = float("inf")
    start_node = None

    for node, edges in graph.items():
        if len(edges) < min_edges:
            min_edges = len(edges)
            start_node = node

    return start_node

def traverse_all_edges(graph, start, total_edges, battery_capacity, recharge):
    """Travels all edges with the least possible cost"""
    visited_edges = set()
    stack = [start]
    path = []
    visited_count = 0 
    current_fuel = battery_capacity
    recharge_count = 0

    while stack:
        curr = stack[-1]
        path.append(curr)
        print("Current node:", curr)

        if visited_count == total_edges or current_fuel <= 0 or recharge_count > recharge:
            break

        for i, (v, is_one_way, length) in enumerate(graph[curr]):
            edge = (min(curr, v), max(curr, v))

            if v == start and current_fuel - length <= 0:
                print("Recharging at node:", curr)
                current_fuel = battery_capacity
                recharge_count += 1
                break

            # Check if the target node is start node and visited_count + battery_capacity < total_edges
            if v == start and visited_count + battery_capacity < total_edges:
                print("Doest not have enough fuel to travel all edges")
                break

            if edge not in visited_edges and current_fuel - length >= 0:
                print("Visiting edge:", edge, visited_edges)
                # check if the all neighbors are visited eg: go to node 1 for edge 1-6 check neighbors of 1
                # Neighbors of 1 : [(0, False, 10), (6, False, 10), (2, False, 10)] - check if all exepct 6 are visited 
                print("Neighbors of", v, ":", graph[v])
                if all((min(v, neighbor), max(v, neighbor)) in visited_edges for neighbor, _, _ in graph[v]):
                    print("All neighbors are visited")
                    break
                visited_edges.add(edge)
                visited_count += 1
                stack.append(v)
                current_fuel -= length
                break
        else:
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

    graph, total_edges = build_graph(data["roads"])
    print("Graph constructed:", dict(graph))
    print("Total number of edges:", total_edges)

    battery_capacity = data["batteryCapacity"]
    num_days = data["numDays"]
    start_node = get_start_node(graph)
    print("Starting node:", start_node)

    path = traverse_all_edges(graph, start_node, total_edges, battery_capacity, num_days - 1)
    print("Traveling all edges:", path)
