import heapq
from collections import deque


# Dijkstra's to find shortest paths in a graph
def dijkstra(graph, start):
    n = len(graph)
    distances = [float('inf')] * n
    distances[start] = 0
    priority_queue = [(0, start)]
    parents = [-1] * n  # To store paths

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        for neighbor, weight in enumerate(graph[current_node]):
            if weight > 0:  
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    parents[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

    return distances, parents


# Dijkstra
def optimal_wiring(graph):
    _, parents = dijkstra(graph, 0)
    wiring = []
    for i in range(1, len(parents)):
        if parents[i] != -1:
            wiring.append((parents[i], i))
    return wiring


# BFS to find an augmenting path for EK
def bfs(residual, source, sink, parent):
    visited = [False] * len(residual)
    queue = deque([source])
    visited[source] = True

    while queue:
        current = queue.popleft()
        for neighbor, capacity_available in enumerate(residual[current]):
            if not visited[neighbor] and capacity_available > 0: 
                parent[neighbor] = current
                if neighbor == sink:
                    return True
                queue.append(neighbor)
                visited[neighbor] = True
    return False


# Edmonds-Karp Algorithm for Maximum Flow
def edmonds_karp(capacity, source, sink):
    n = len(capacity)
    residual = [row[:] for row in capacity]
    parent = [-1] * n
    max_flow = 0

    while bfs(residual, source, sink, parent):
        # Find bottleneck capacity
        path_flow = float('inf')
        current = sink
        while current != source:
            path_flow = min(path_flow, residual[parent[current]][current])
            current = parent[current]

        # Update residual capacities
        current = sink
        while current != source:
            prev = parent[current]
            residual[prev][current] -= path_flow
            residual[current][prev] += path_flow
            current = prev

        max_flow += path_flow

    return max_flow


def main():

    input_file = input("Enter the input file name: ")
    with open(input_file, 'r') as file:
        n = int(file.readline().strip())  
        distance_matrix = [list(map(int, file.readline().strip().split())) for _ in range(n)]
        capacity_matrix = [list(map(int, file.readline().strip().split())) for _ in range(n)]

    # Dijkstra
    wiring = optimal_wiring(distance_matrix)
    print("\nOptimal wiring (fiber connections):")
    print(wiring)

    # Edmonds-Karp
    source = 0
    sink = n - 1
    max_flow = edmonds_karp(capacity_matrix, source, sink)
    print(f"\nMaximum Information Flow: {max_flow}")


if __name__ == "__main__":
    main()