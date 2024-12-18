from collections import defaultdict

def build_graph(roads):
    """Construct the graph with directed and undirected edges."""
    graph = defaultdict(list)
    total_edges = 0

    for road in roads:
        u, v, is_one_way, length = road["intersectionId1"], road["intersectionId2"], road["isOneWay"], road["length"]
        graph[u].append((v, length))
        total_edges += 1
        if not is_one_way:
            graph[v].append((u, length))

    return graph, total_edges

def traverse_all_edges(graph, start, total_edges, battery_capacity):
    """Travels all edges with the least possible cost"""
    visited_edges = set()
    stack = [start]
    path = []
    visited_count = 0 
    current_fuel = battery_capacity

    while stack:
        u = stack[-1]
        path.append(u)

        if visited_count == total_edges or current_fuel <= 0: 
            break

        for i, (v, length) in enumerate(graph[u]):
            edge = (min(u, v), max(u, v))

            if edge not in visited_edges and current_fuel - length >= 0:
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

    battery_capacity = data["batteryCapacity"] * data["numDays"]

    start_node = 0
    path = traverse_all_edges(graph, start_node, total_edges, battery_capacity)
    print("Traveling all edges:", path)
