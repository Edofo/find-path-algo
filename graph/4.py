import networkx as nx
import matplotlib.pyplot as plt
import json

# Charger les données
with open("datasets/4_manhattan.json", "r") as file:
    data = json.load(file)

# Créer un graphe à partir des données
G = nx.DiGraph() if any(road["isOneWay"] for road in data["roads"]) else nx.Graph()

# Ajouter les nœuds
for intersection in data["intersections"]:
    G.add_node(intersection["id"], pos=(intersection["lng"], intersection["lat"]))

# Ajouter les arêtes
for road in data["roads"]:
    G.add_edge(road["intersectionId1"], road["intersectionId2"], length=road["length"])
    if not road["isOneWay"]:
        G.add_edge(road["intersectionId2"], road["intersectionId1"], length=road["length"])

# Visualisation du graphe
plt.figure(figsize=(12, 8))
pos = nx.get_node_attributes(G, 'pos')
nx.draw(G, pos, with_labels=True, node_size=50, font_size=8)
plt.title("Carte de Pac-Man")
plt.show()

# Analyse de base
print("Nombre de nœuds :", G.number_of_nodes())
print("Nombre d'arêtes :", G.number_of_edges())
print("Le graphe est-il connexe ?", nx.is_connected(G.to_undirected()))
print("Le graphe contient-il des cycles ?", nx.is_directed_acyclic_graph(G))
