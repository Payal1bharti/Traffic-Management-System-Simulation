#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <stack>

using namespace std;

class Graph {
private:
    unordered_map<int, vector<pair<int, int>>> adj; // Adjacency list (Node -> [(Neighbor, Weight)])

public:
    void addRoad(int u, int v, int weight) {
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight}); // Undirected Graph (Two-way road)
    }

    void displayGraph() {
        cout << "\n🚦 City Road Network:\n";
        for (auto &node : adj) {
            cout << "Location " << node.first << " -> ";
            for (auto &neighbor : node.second) {
                cout << "(" << neighbor.first << ", Distance: " << neighbor.second << ") ";
            }
            cout << endl;
        }
    }

    void shortestPath(int start, int destination) {
        unordered_map<int, int> dist;
        unordered_map<int, int> parent;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

        for (auto &node : adj) {
            dist[node.first] = numeric_limits<int>::max();
        }

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto &neighbor : adj[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        if (dist[destination] == numeric_limits<int>::max()) {
            cout << "\n❌ No path exists between " << start << " and " << destination << ".\n";
            return;
        }

        cout << "\n✅ Shortest Path from " << start << " to " << destination << ": ";
        int current = destination;
        stack<int> path;
        while (current != start) {
            path.push(current);
            current = parent[current];
        }
        path.push(start);

        while (!path.empty()) {
            cout << path.top();
            path.pop();
            if (!path.empty()) cout << " -> ";
        }
        cout << "\n📏 Total Distance: " << dist[destination] << " units\n";
    }

    void bfs(int start) {
        unordered_map<int, bool> visited;
        queue<int> q;

        q.push(start);
        visited[start] = true;

        cout << "\n🌍 BFS Traffic Flow from Location " << start << ": ";
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            cout << u << " ";

            for (auto &neighbor : adj[u]) {
                int v = neighbor.first;
                if (!visited[v]) {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }
        cout << endl;
    }

    void dfsHelper(int node, unordered_map<int, bool> &visited) {
        cout << node << " ";
        visited[node] = true;

        for (auto &neighbor : adj[node]) {
            if (!visited[neighbor.first]) {
                dfsHelper(neighbor.first, visited);
            }
        }
    }

    void dfs(int start) {
        unordered_map<int, bool> visited;
        cout << "\n🛣️ DFS Road Analysis from Location " << start << ": ";
        dfsHelper(start, visited);
        cout << endl;
    }
};

int main() {
    Graph cityTraffic;
    int locations, roads;

    cout << "🏙️ Enter the number of locations in the city: ";
    cin >> locations;

    cout << "🛣️ Enter the number of roads: ";
    cin >> roads;

    cout << "\n🔗 Enter roads in the format (Location1 Location2 Distance):\n";
    for (int i = 0; i < roads; i++) {
        int u, v, weight;
        cin >> u >> v >> weight;
        cityTraffic.addRoad(u, v, weight);
    }

    cityTraffic.displayGraph();

    int start, destination;
    cout << "\n🚗 Enter the source and destination to find the shortest path: ";
    cin >> start >> destination;
    cityTraffic.shortestPath(start, destination);

    int bfsStart;
    cout << "\n🚦 Enter the location for BFS Traffic Flow analysis: ";
    cin >> bfsStart;
    cityTraffic.bfs(bfsStart);

    int dfsStart;
    cout << "\n🛤️ Enter the location for DFS Road Analysis: ";
    cin >> dfsStart;
    cityTraffic.dfs(dfsStart);

    return 0;
}
