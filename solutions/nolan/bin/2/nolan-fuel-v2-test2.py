from collections import defaultdict, deque
import json

DATASET_FILE = 'datasets/2_pacman.json'

def build_graph(intersections, roads):
    """
    Construit un graphe sous forme de liste d'adjacence.
    
    :param intersections: Liste des intersections avec leurs coordonnées.
    :param roads: Liste des routes reliant les intersections.
    :return: Graphe sous forme de dictionnaire (liste d'adjacence) et un dictionnaire des coordonnées.
    """
    graph = defaultdict(list)
    coordinates = {}
    
    # Charger les intersections
    for intersection in intersections:
        node_id = intersection['id']
        coordinates[node_id] = (intersection['lat'], intersection['lng'])
    
    # Charger les routes
    for road in roads:
        start, end, is_one_way = road['intersectionId1'], road['intersectionId2'], road['isOneWay']
        graph[start].append((end, road['length']))
        if not is_one_way:
            graph[end].append((start, road['length']))
    
    return graph, coordinates

def find_best_station(graph, coordinates):
    """
    Identifie la meilleure position pour la station de recharge.
    
    :param graph: Liste d'adjacence représentant le graphe.
    :param coordinates: Dictionnaire des coordonnées des intersections {id: (lat, lng)}.
    :return: ID de l'intersection choisie comme station de recharge.
    """
    import math

    def euclidean_distance(coord1, coord2):
        """Calcule la distance euclidienne entre deux points."""
        return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

    # Calculer les métriques pour chaque intersection
    scores = {}
    for node, neighbors in graph.items():
        degree = len(neighbors)  # Degré (nombre de connexions)
        
        # Distance moyenne par rapport à toutes les autres intersections
        total_distance = 0
        for other_node, coord in coordinates.items():
            if node != other_node:
                total_distance += euclidean_distance(coordinates[node], coord)
        avg_distance = total_distance / (len(coordinates) - 1)
        
        # Score combiné : privilégier les intersections très connectées et centrales
        scores[node] = {
            'degree': degree,
            'avg_distance': avg_distance,
            'score': degree / avg_distance  # Ratio connectivité/distance moyenne
        }

    # Trouver le nœud avec le meilleur score
    best_station = max(scores, key=lambda node: scores[node]['score'])
    
    return best_station, scores[best_station]

from collections import deque

def find_shortest_path(graph, start, goal):
    """
    Trouve le chemin le plus court entre deux nœuds dans un graphe non pondéré.
    """
    queue = deque([(start, [start])])  # File d'attente avec le chemin parcouru
    visited = set()

    while queue:
        current_node, path = queue.popleft()
        if current_node == goal:
            print(f"Chemin trouvé : {path}")
            return path  # Retourner le chemin complet

        if current_node not in visited:
            visited.add(current_node)
            for neighbor, _ in graph[current_node]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None  # Aucun chemin trouvé


def plan_robot_path(graph, station, battery_capacity, num_days):
    """
    Planifie un chemin pour maximiser les routes couvertes, tout en respectant les contraintes.
    
    :param graph: Graphe sous forme de liste d'adjacence.
    :param station: Intersection où se trouve la station de recharge.
    :param battery_capacity: Capacité maximale de la batterie.
    :param num_days: Nombre total de jours disponibles.
    :return: Chemin optimisé couvrant un maximum de routes.
    """
    visited_edges = set()  # Ensemble des arêtes parcourues
    path = []  # Chemin global
    current_day = 1  # Jour actuel
    remaining_battery = battery_capacity  # Batterie restante

    def dfs(node):
        nonlocal current_day, remaining_battery, visited_edges, path

        for neighbor, length in graph[node]:
            edge = (min(node, neighbor), max(node, neighbor))
            print(f"Current node: {node}, Neighbor: {neighbor}, Edge: {edge}")

            edges = graph[node]
            is_last_edge = neighbor == edges[-1][0] and station == node
            
            if edge not in visited_edges and remaining_battery >= length or is_last_edge:
                # Parcourir l'arête
                visited_edges.add(edge)
                path.append((node, neighbor))
                remaining_battery -= length
                dfs(neighbor)

        # Si la batterie est faible, retourner à la station
        if remaining_battery < battery_capacity // 2 and current_day < num_days:
            shortest_path = find_shortest_path(graph, node, station)
            if not shortest_path:
                raise ValueError(f"Aucun chemin valide de {node} à {station}")

            # Ajouter le chemin vers la station
            for i in range(len(shortest_path) - 1):
                start = shortest_path[i]
                end = shortest_path[i + 1]
                edge = (min(start, end), max(start, end))
                visited_edges.add(edge)
                path.append((start, end))
                remaining_battery -= dict(graph[start])[end]

            # Recharger la batterie
            remaining_battery = battery_capacity
            current_day += 1

            # Reprendre depuis la station
            dfs(station)

    # Démarrer depuis la station
    dfs(station)

    return path


with open(DATASET_FILE) as fi:
    dataset = fi.read()
data = json.loads(dataset)

# Construire le graphe
graph, coordinates = build_graph(data["intersections"], data["roads"])

# Identifier la meilleure station de recharge
best_station, metrics = find_best_station(graph, coordinates)

print(f"Meilleure station : {best_station}")
print(f"Métriques associées : {metrics}")

# Planifier le chemin optimal pour le robot
robot_path = plan_robot_path(graph, best_station, data["batteryCapacity"], data["numDays"])
print(f"Chemin optimal : {robot_path}")

def convert_to_json(station_id, path):
    """
    Convertit le chemin optimal en un format JSON avec la station de recharge et l'itinéraire.
    
    :param station_id: ID de la station de recharge.
    :param path: Liste des tuples représentant le chemin [(nœud1, nœud2), ...].
    :return: JSON au format demandé.
    """
    # Extraire les intersections dans l'ordre visité à partir des paires (nœud1, nœud2)
    itinerary = [station_id]  # Initialiser l'itinéraire avec la station de départ
    for edge in path:
        if edge[1] != itinerary[-1]:  # Ajouter le prochain nœud s'il diffère du précédent
            itinerary.append(edge[1])
    
    # Construire le dictionnaire au format demandé
    result = {
        "chargeStationId": station_id,
        "itinerary": itinerary
    }
    
    # Convertir en JSON
    return json.dumps(result, indent=2)

# Convertir le résultat en JSON
json_result = convert_to_json(best_station, robot_path)
# with open('output/2_submission.json', 'w') as f:
#     f.write(json_result)
