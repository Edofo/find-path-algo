from collections import defaultdict

def build_graph(intersections, roads):
    """Construit le graphe avec les arêtes orientées et non orientées."""
    graph = defaultdict(list)
    total_edges = 0  # Compteur pour le nombre total d'arêtes

    for road in roads:
        u, v, is_one_way = road["intersectionId1"], road["intersectionId2"], road["isOneWay"]
        graph[u].append((v, is_one_way))
        total_edges += 1
        if not is_one_way:
            graph[v].append((u, is_one_way))

    return graph, total_edges

def traverse_all_edges(graph, start, total_edges):
    """Parcourt toutes les arêtes exactement une fois."""
    visited_edges = set()  # Pour marquer les arêtes visitées
    stack = [start]  # Pile pour le parcours
    path = []  # Chemin final
    visited_count = 0  # Compteur des arêtes visitées

    while stack:
        u = stack[-1]
        path.append(u)

        if visited_count == total_edges:  # Arrêter quand toutes les arêtes sont visitées
            break

        for i, (v, is_one_way) in enumerate(graph[u]):
            # Identifier une arête unique
            if is_one_way:
                edge = (u, v)  # Pour les arêtes orientées
            else:
                edge = (min(u, v), max(u, v))  # Pour les arêtes non orientées

            if edge not in visited_edges:  # Si l'arête n'a pas encore été visitée
                visited_edges.add(edge)
                visited_count += 1
                stack.append(v)
                break
        else:   
            stack.pop()  # Retour en arrière si aucun voisin non visité

    return path

# Fonction principale
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

    # Construire le graphe
    graph, total_edges = build_graph(data["intersections"], data["roads"])
    print("Graphe construit :", dict(graph))
    print("Nombre total d'arêtes :", total_edges)

    # Parcourir toutes les arêtes
    start_node = 0
    path = traverse_all_edges(graph, start_node, total_edges)
    print("Parcours de toutes les arêtes :", path)
