#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include "json.hpp"

using json = nlohmann::json;

int main() {
    std::ifstream input("graph_data.json");
    json graphData;
    input >> graphData;

    json verticesArray = graphData["vertices"];
    for (const auto& vertex : verticesArray) {
        int id = vertex["id"];
        std::string name = vertex["name"];
        std::cout << "Vertex ID: " << id << ", Name: " << name << std::endl;
    }

    json arcsArray = graphData["arcs"];
    for (const auto& arc : arcsArray) {
        int source = arc["source"];
        int destination = arc["destination"];
        int weight = arc["weight"];
        std::cout << "Arc Source: " << source << ", Destination: "
            << destination << ", Weight: " << weight << std::endl;
    }

    return 0;
}





const int MAX_VERTICES = 100;

class Graph {
private:
    int vertices;
    int adjMatrix[MAX_VERTICES][MAX_VERTICES];
    std::vector<int> adjList[MAX_VERTICES];

public:
    Graph(int numVertices) : vertices(numVertices) {
        for (int i = 0; i < vertices; ++i) {
            for (int j = 0; j < vertices; ++j) {
                adjMatrix[i][j] = 0;
            }
        }
    }

    void addEdge(int src, int dest, int weight) {
        adjMatrix[src][dest] = weight;
        adjList[src].push_back(dest);
    }

};




void dijkstraShortestPath(const Graph& graph, int startVertex, int endVertex) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
        std::greater<std::pair<int, int>>> pq;
    std::vector<int> dist(graph.vertices, std::numeric_limits<int>::max());
    std::vector<bool> visited(graph.vertices, false);

    pq.push({0, startVertex});
    dist[startVertex] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == endVertex) break;

        if (visited[u]) continue;
        visited[u] = true;

        for (int v : graph.adjList[u]) {
            if (!visited[v] && dist[u] + graph.adjMatrix[u][v] < dist[v]) {
                dist[v] = dist[u] + graph.adjMatrix[u][v];
                pq.push({dist[v], v});
            }
        }
    }

    std::cout << "Shortest distance from vertex " << startVertex <<
        " to vertex " << endVertex << " is " << dist[endVertex] << std::endl;
}

