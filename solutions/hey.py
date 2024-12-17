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


"""
-> convert the the current data to a graph
-> traverse the graph using BFS or DFS 
"""

def build_graph(intersections, roads):
    graph = {}

    for inter in intersections:
        id = inter["id"]
        graph[id] = []
    
    for road in roads:
        src = road.get("intersectionId1")
        to = road.get("intersectionId2")

        graph[src].append(to)
        if not road["isOneWay"]:
            graph[to].append(src)

    return graph

graph = build_graph(data["intersections"], data["roads"])
print(graph)

def dfs_edges(graph, start):
    visited = set()
    stack = [( (start, 70) )]
    edges = []

    while stack:
        current, fuel = stack.pop()
        for neighbor in graph[current]:
            pos = (current, neighbor)
            reverse_pos = (neighbor, current)

            if pos not in visited and reverse_pos not in visited:
                visited.add(pos)
                visited.add(reverse_pos)
                stack.append((neighbor, fuel - 10))
                edges.append(pos)

    return edges

print(dfs_edges(graph, 0))

def dfs(graph, start):
    visited = set()
    stack = [start]

    while stack:
        current = stack.pop()
        for neighbor in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                stack.append(neighbor)

    return visited

print(dfs(graph, 0))