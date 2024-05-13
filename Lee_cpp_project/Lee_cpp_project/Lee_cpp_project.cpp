
#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <queue>
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

const int INF = std::numeric_limits<int>::max();

// Function to read the JSON file
json readJsonFile(const std::string& filename) {
    std::ifstream file(filename);
    json j;
    file >> j;
    return j;
}

// Graph using adjacency matrix
class GraphMatrix {
public:
    GraphMatrix(int n) : n(n) {
        matrix.resize(n, std::vector<int>(n, INF));
        for (int i = 0; i < n; ++i) {
            matrix[i][i] = 0;
        }
    }

    void addEdge(int u, int v, int weight) {
        matrix[u][v] = weight;
    }

    std::vector<int> dijkstra(int src, int dest) {
        std::vector<int> dist(n, INF);
        std::vector<int> parent(n, -1);
        std::vector<bool> visited(n, false);
        dist[src] = 0;

        for (int i = 0; i < n; ++i) {
            int u = -1;
            for (int j = 0; j < n; ++j) {
                if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                    u = j;
                }
            }

            if (dist[u] == INF) break;

            visited[u] = true;

            for (int v = 0; v < n; ++v) {
                if (matrix[u][v] != INF && dist[u] + matrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + matrix[u][v];
                    parent[v] = u;
                }
            }
        }

        std::vector<int> path;
        for (int v = dest; v != -1; v = parent[v]) {
            path.push_back(v);
        }
        std::reverse(path.begin(), path.end());

        if (path.size() == 1 && path[0] != src) {
            path.clear();
        }

        return path;
    }

private:
    int n;
    std::vector<std::vector<int>> matrix;
};

// Graph using adjacency list
struct Edge {
    int v, weight;
    Edge* next;
};

class GraphList {
public:
    GraphList(int n) : n(n) {
        adjList.resize(n, nullptr);
    }

    void addEdge(int u, int v, int weight) {
        Edge* edge = new Edge{ v, weight, adjList[u] };
        adjList[u] = edge;
    }

    std::vector<int> dijkstra(int src, int dest) {
        std::vector<int> dist(n, INF);
        std::vector<int> parent(n, -1);
        std::vector<bool> visited(n, false);
        dist[src] = 0;

        using Node = std::pair<int, int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        pq.push({ 0, src });

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (visited[u]) continue;
            visited[u] = true;

            for (Edge* edge = adjList[u]; edge != nullptr; edge = edge->next) {
                int v = edge->v;
                int weight = edge->weight;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push({ dist[v], v });
                }
            }
        }

        std::vector<int> path;
        for (int v = dest; v != -1; v = parent[v]) {
            path.push_back(v);
        }
        std::reverse(path.begin(), path.end());

        if (path.size() == 1 && path[0] != src) {
            path.clear();
        }

        return path;
    }

private:
    int n;
    std::vector<Edge*> adjList;
};

// Utility function to print the path
void printPath(const std::vector<int>& path, const std::unordered_map<int, std::string>& vertexMap) {
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
        return;
    }
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << vertexMap.at(path[i]);
        if (i != path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}

int main() {
    // Read JSON file
    try {
        json j = readJsonFile("graph.json");
        std::vector<std::string> vertices = j["vertices"];
        auto arcs = j["arcs"];

        std::unordered_map<std::string, int> vertexIndex;
        std::unordered_map<int, std::string> indexVertex;
        int n = vertices.size();

        for (int i = 0; i < n; ++i) {
            vertexIndex[vertices[i]] = i;
            indexVertex[i] = vertices[i];
        }

        // Create graph using adjacency matrix
        GraphMatrix graphMatrix(n);
        // Create graph using adjacency list
        GraphList graphList(n);

        for (const auto& arc : arcs) {
            int u = vertexIndex[arc["from"]];
            int v = vertexIndex[arc["to"]];
            int weight = arc["weight"];
            graphMatrix.addEdge(u, v, weight);
            graphList.addEdge(u, v, weight);
        }

        // Input two vertices to find the shortest route
        std::string startVertex, endVertex;
        std::cout << "Enter start vertex: ";
        std::cin >> startVertex;
        std::cout << "Enter end vertex: ";
        std::cin >> endVertex;

        int start = vertexIndex[startVertex];
        int end = vertexIndex[endVertex];

        // Find and print shortest path using adjacency matrix
        std::cout << "Shortest path using adjacency matrix: ";
        std::vector<int> pathMatrix = graphMatrix.dijkstra(start, end);
        printPath(pathMatrix, indexVertex);

        // Find and print shortest path using adjacency list
        std::cout << "Shortest path using adjacency list: ";
        std::vector<int> pathList = graphList.dijkstra(start, end);
        printPath(pathList, indexVertex);
        std::cin >> startVertex;
    }
    catch (std::exception& e) {
        std::cout << "Exception: " << e.what() << '\n';
    }

    
    return 0;
}
