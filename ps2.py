"""
@description: Integrative activity 2, finding optimal route and maximum possible flow
@authors: Aracelli Melissa Boza Zabarburú A01662934, Adolfo Hernández Fernández A01664412, Luis Enrique Salazar Pérez A00833460
@date: 2024/25/11
"""

import heapq
from collections import deque

# Dijkstra's to find shortest paths in a graph
def dijkstra(graph, start):
    n = len(graph)  # node count
    distances = [float('inf')] * n  # Infinity ensures initial comparisons favor updates
    distances[start] = 0 
    priority_queue = [(0, start)]  # Tracks nodes by smallest known distance
    parents = [-1] * n 

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        for neighbor, weight in enumerate(graph[current_node]):
            if weight > 0:  # Valid edge exists
                distance = current_distance + weight
                if distance < distances[neighbor]:  # Shorter path found
                    distances[neighbor] = distance
                    parents[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))  # Maintain priority order

    return parents

# Dijkstra
def optimal_wiring(graph):
    parents = dijkstra(graph, 0)
    wiring = []  # Stores connections for minimum spanning-like result
    for i in range(1, len(parents)):
        if parents[i] != -1:  # Valid connection exists
            wiring.append((parents[i], i))  # Record edge
    return wiring

# BFS to find an augmenting path for EK
def bfs(residual, source, sink, parent):
    visited = [False] * len(residual)  # Avoid revisiting nodes
    queue = deque([source])  # FIFO structure for layer-based exploration
    visited[source] = True  # Mark the start node

    while queue:
        current = queue.popleft()
        for neighbor, capacity_available in enumerate(residual[current]):
            if not visited[neighbor] and capacity_available > 0:  # Only consider viable edges
                parent[neighbor] = current  # Record path
                if neighbor == sink:  # Path to sink found
                    return True
                queue.append(neighbor)  # Explore next
                visited[neighbor] = True  # Mark as visited
    return False

# Edmonds-Karp Algorithm for Maximum Flow
def edmonds_karp(capacity, source, sink):
    n = len(capacity)
    residual = [row[:] for row in capacity]
    parent = [-1] * n
    max_flow = 0

    while bfs(residual, source, sink, parent):
        path_flow = float('inf')  # Start with unbounded capacity
        current = sink
        while current != source:  # Trace path back to source
            path_flow = min(path_flow, residual[parent[current]][current])
            current = parent[current]

        current = sink
        while current != source:  # Update residual capacities
            prev = parent[current]
            residual[prev][current] -= path_flow  # Reduce forward capacity
            residual[current][prev] += path_flow  # Add backward capacity
            current = prev

        max_flow += path_flow  # Accumulate total flow

    return max_flow

def main():
    input_file = input("Enter the input file name: ")
    with open(input_file, 'r') as file:
        n = int(file.readline().strip())  # Read number of nodes (matrix size)
        distance_matrix = [list(map(int, file.readline().strip().split())) for _ in range(n)]
        capacity_matrix = [list(map(int, file.readline().strip().split())) for _ in range(n)]

    wiring = optimal_wiring(distance_matrix)
    print("\nOptimal wiring (fiber connections):")
    print(wiring)

    source = 0
    sink = n - 1
    max_flow = edmonds_karp(capacity_matrix, source, sink)
    print(f"\nMaximum Information Flow: {max_flow}")

if __name__ == "__main__":
    main()