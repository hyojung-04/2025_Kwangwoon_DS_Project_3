#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

#include "Graph.h"
#include "GraphMethod.h"

using namespace std;

#define INF 100000000 // Value representing infinity

// ==========================================================
// Helper Function: Convert Graph to Adjacency List
// ==========================================================
// This function helps to traverse the graph easily.
// It retrieves edges using the Graph class interface.
vector<vector<pair<int, int>>> getAdjacencyList(Graph *graph, char option) {
  int size = graph->getSize();
  vector<vector<pair<int, int>>> adj(size);

  for (int i = 0; i < size; i++) {
    map<int, int> m;

    // Get edges based on the option (Directed 'O' or Undirected 'X')
    if (option == 'O') {
      graph->getAdjacentEdgesDirect(i, &m); // Outgoing edges only
    } else {
      graph->getAdjacentEdges(i, &m); // Both outgoing and incoming
    }

    // Move edges from map to vector
    for (auto iter = m.begin(); iter != m.end(); iter++) {
      adj[i].push_back({iter->first, iter->second});
    }

    // Sort neighbors by vertex index (ascending)
    // This ensures we visit the smallest index node first.
    sort(adj[i].begin(), adj[i].end());
  }

  return adj;
}

// ==========================================================
// BFS (Breadth-First Search)
// ==========================================================
bool BFS(Graph *graph, char option, int vertex) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  // Check if vertex is valid
  if (vertex < 0 || vertex >= graph->getSize()) {
    fout.close();
    return false;
  }

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  vector<bool> visited(graph->getSize(), false);
  queue<int> q;

  // Print Algorithm Name
  if (option == 'O')
    fout << "Directed Graph BFS" << endl;
  else
    fout << "Undirected Graph BFS" << endl;

  fout << "Start: " << vertex << endl;

  // Start BFS
  q.push(vertex);
  visited[vertex] = true;

  bool first = true;
  while (!q.empty()) {
    int curr = q.front();
    q.pop();

    // Print format: v -> v -> v
    if (!first)
      fout << " -> ";
    fout << curr;
    first = false;

    // Visit neighbors
    for (auto &edge : adj[curr]) {
      int next = edge.first;
      if (!visited[next]) {
        visited[next] = true;
        q.push(next);
      }
    }
  }
  fout << endl; // End line
  fout.close();
  return true;
}

// ==========================================================
// DFS (Depth-First Search)
// ==========================================================
bool DFS(Graph *graph, char option, int vertex) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  if (vertex < 0 || vertex >= graph->getSize()) {
    fout.close();
    return false;
  }

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  vector<bool> visited(graph->getSize(), false);
  stack<int> s;

  if (option == 'O')
    fout << "Directed Graph DFS" << endl;
  else
    fout << "Undirected Graph DFS" << endl;

  fout << "Start: " << vertex << endl;

  // To visit smaller index neighbors first using a Stack,
  // we must push them in reverse order (descending).
  for (auto &vec : adj) {
    sort(vec.rbegin(), vec.rend());
  }

  s.push(vertex);

  bool first = true;
  while (!s.empty()) {
    int curr = s.top();
    s.pop();

    if (visited[curr])
      continue;
    visited[curr] = true;

    if (!first)
      fout << " -> ";
    fout << curr;
    first = false;

    for (auto &edge : adj[curr]) {
      int next = edge.first;
      if (!visited[next]) {
        s.push(next);
      }
    }
  }

  fout << endl;
  fout.close();
  return true;
}

// ==========================================================
// Kruskal's Algorithm (MST)
// ==========================================================
// Struct for Edge
struct Edge {
  int u, v, weight;
  // Operator overloading for sorting
  bool operator<(const Edge &other) const {
    if (weight != other.weight)
      return weight < other.weight;
    if (u != other.u)
      return u < other.u;
    return v < other.v;
  }
};

// Disjoint Set (Union-Find)
struct DisjointSet {
  vector<int> parent;
  DisjointSet(int n) {
    parent.resize(n);
    for (int i = 0; i < n; i++)
      parent[i] = i;
  }
  // Find with path compression
  int find(int u) {
    if (u == parent[u])
      return u;
    return parent[u] = find(parent[u]);
  }
  // Union
  void merge(int u, int v) {
    u = find(u);
    v = find(v);
    if (u != v)
      parent[u] = v;
  }
};

bool Kruskal(Graph *graph) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  // Kruskal works on Undirected graphs ('X')
  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, 'X');
  vector<Edge> edges;
  int size = graph->getSize();

  // Collect all unique edges (u < v to avoid duplicates)
  for (int u = 0; u < size; u++) {
    for (auto &p : adj[u]) {
      int v = p.first;
      int w = p.second;
      if (u < v) {
        edges.push_back({u, v, w});
      }
    }
  }

  // Sort edges by weight (Ascending)
  sort(edges.begin(), edges.end());

  DisjointSet ds(size);
  vector<Edge> mst;
  int costSum = 0;

  // Pick edges
  for (auto &e : edges) {
    if (ds.find(e.u) != ds.find(e.v)) { // If no cycle
      ds.merge(e.u, e.v);
      mst.push_back(e);
      costSum += e.weight;
    }
  }

  // Check if MST covers all vertices (V-1 edges)
  // Exception: A single node graph (V=1, E=0) is connected.
  if (size > 1 && mst.size() < size - 1) {
    fout.close();
    return false; // Error: Graph is disconnected
  }

  // Prepare for printing (Rebuild adjacency list for MST)
  vector<vector<pair<int, int>>> mstAdj(size);
  for (auto &e : mst) {
    mstAdj[e.u].push_back({e.v, e.weight});
    mstAdj[e.v].push_back({e.u, e.weight});
  }

  // Print MST
  for (int i = 0; i < size; i++) {
    sort(mstAdj[i].begin(), mstAdj[i].end());
    fout << "[" << i << "] ";
    for (auto &p : mstAdj[i]) {
      fout << p.first << "(" << p.second << ") ";
    }
    fout << endl;
  }
  fout << "Cost: " << costSum << endl;

  fout.close();
  return true;
}

// ==========================================================
// Dijkstra's Algorithm
// ==========================================================
bool Dijkstra(Graph *graph, char option, int vertex) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  if (vertex < 0 || vertex >= graph->getSize()) {
    fout.close();
    return false;
  }

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  int size = graph->getSize();

  // Check for negative weights
  for (int i = 0; i < size; i++) {
    for (auto &p : adj[i]) {
      if (p.second < 0) {
        fout.close();
        return false; // Error code 600
      }
    }
  }

  // Init distances
  vector<int> dist(size, INF);
  vector<int> parent(size, -1);
  // Min-heap: {distance, vertex}
  priority_queue<pair<int, int>, vector<pair<int, int>>,
                 greater<pair<int, int>>>
      pq;

  dist[vertex] = 0;
  pq.push({0, vertex});

  while (!pq.empty()) {
    int currDist = pq.top().first;
    int curr = pq.top().second;
    pq.pop();

    if (currDist > dist[curr])
      continue;

    for (auto &edge : adj[curr]) {
      int next = edge.first;
      int weight = edge.second;

      // Relaxation
      if (dist[curr] + weight < dist[next]) {
        dist[next] = dist[curr] + weight;
        parent[next] = curr;
        pq.push({dist[next], next});
      }
    }
  }

  if (option == 'O')
    fout << "Directed Graph Dijkstra" << endl;
  else
    fout << "Undirected Graph Dijkstra" << endl;

  fout << "Start: " << vertex << endl;

  for (int i = 0; i < size; i++) {
    if (i == vertex)
      continue; // Skip start node

    fout << "[" << i << "] ";
    if (dist[i] == INF) {
      fout << "x" << endl;
    } else {
      // Reconstruct Path
      vector<int> path;
      int curr = i;
      while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
      }
      // Print Path
      for (int k = path.size() - 1; k >= 0; k--) {
        fout << path[k];
        if (k > 0)
          fout << " -> ";
      }
      fout << " (" << dist[i] << ")" << endl;
    }
  }

  fout.close();
  return true;
}

// ==========================================================
// Bellman-Ford Algorithm
// ==========================================================
bool Bellmanford(Graph *graph, char option, int s_vertex, int e_vertex) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  int size = graph->getSize();
  if (s_vertex < 0 || s_vertex >= size || e_vertex < 0 || e_vertex >= size) {
    fout.close();
    return false;
  }

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  vector<int> dist(size, INF);
  vector<int> parent(size, -1);

  dist[s_vertex] = 0;

  // Relaxation V-1 times
  for (int i = 0; i < size - 1; i++) {
    for (int u = 0; u < size; u++) {
      if (dist[u] == INF)
        continue;
      for (auto &edge : adj[u]) {
        int v = edge.first;
        int w = edge.second;
        if (dist[u] + w < dist[v]) {
          dist[v] = dist[u] + w;
          parent[v] = u;
        }
      }
    }
  }

  // Check Negative Cycle
  for (int u = 0; u < size; u++) {
    if (dist[u] == INF)
      continue;
    for (auto &edge : adj[u]) {
      int v = edge.first;
      int w = edge.second;
      if (dist[u] + w < dist[v]) {
        fout.close();
        return false; // Error code 700 (Negative Cycle)
      }
    }
  }

  if (option == 'O')
    fout << "Directed Graph Bellman-Ford" << endl;
  else
    fout << "Undirected Graph Bellman-Ford" << endl;

  if (dist[e_vertex] == INF) {
    fout << "x" << endl;
  } else {
    vector<int> path;
    int curr = e_vertex;
    while (curr != -1) {
      path.push_back(curr);
      curr = parent[curr];
    }
    for (int k = path.size() - 1; k >= 0; k--) {
      fout << path[k];
      if (k > 0)
        fout << " -> ";
    }
    fout << endl;
    fout << "Cost: " << dist[e_vertex] << endl;
  }

  fout.close();
  return true;
}

// ==========================================================
// Floyd-Warshall Algorithm
// ==========================================================
bool FLOYD(Graph *graph, char option) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  int size = graph->getSize();

  // Init Matrix
  vector<vector<int>> d(size, vector<int>(size, INF));

  for (int i = 0; i < size; i++)
    d[i][i] = 0;

  // Fill edges
  for (int u = 0; u < size; u++) {
    for (auto &edge : adj[u]) {
      int v = edge.first;
      int w = edge.second;
      if (w < d[u][v])
        d[u][v] = w; // Keep min weight
    }
  }

  // Floyd Algorithm
  for (int k = 0; k < size; k++) {
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (d[i][k] != INF && d[k][j] != INF) {
          if (d[i][k] + d[k][j] < d[i][j]) {
            d[i][j] = d[i][k] + d[k][j];
          }
        }
      }
    }
  }

  // Check Negative Cycle (Diagonal check)
  for (int i = 0; i < size; i++) {
    if (d[i][i] < 0) {
      fout.close();
      return false; // Error code 800
    }
  }

  if (option == 'O')
    fout << "Directed Graph Floyd" << endl;
  else
    fout << "Undirected Graph Floyd" << endl;

  // Print Header
  fout << "\t";
  for (int i = 0; i < size; i++)
    fout << "[" << i << "]"
         << "\t";
  fout << endl;

  // Print Body
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "]"
         << "\t";
    for (int j = 0; j < size; j++) {
      if (d[i][j] == INF)
        fout << "x\t";
      else
        fout << d[i][j] << "\t";
    }
    fout << endl;
  }

  fout.close();
  return true;
}

// ==========================================================
// Closeness Centrality
// ==========================================================
bool Centrality(Graph *graph) {
  ofstream fout;
  fout.open("log.txt", ios::app);

  // Reuse Floyd logic for Undirected graph ('X')
  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, 'X');
  int size = graph->getSize();
  vector<vector<int>> d(size, vector<int>(size, INF));

  for (int i = 0; i < size; i++)
    d[i][i] = 0;
  for (int u = 0; u < size; u++) {
    for (auto &edge : adj[u]) {
      int v = edge.first;
      int w = edge.second;
      if (w < d[u][v])
        d[u][v] = w;
    }
  }

  // Run Floyd
  for (int k = 0; k < size; k++) {
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (d[i][k] != INF && d[k][j] != INF) {
          if (d[i][k] + d[k][j] < d[i][j]) {
            d[i][j] = d[i][k] + d[k][j];
          }
        }
      }
    }
  }

  // Check negative cycle
  for (int i = 0; i < size; i++) {
    if (d[i][i] < 0) {
      fout.close();
      return false; // Error code 900
    }
  }

  // Calculate Centrality
  vector<double> values(size, 0.0);
  vector<long long> sums(size, 0);
  double maxVal = -1.0;

  for (int i = 0; i < size; i++) {
    long long sum = 0;
    bool disconnected = false;
    for (int j = 0; j < size; j++) {
      if (i == j)
        continue;
      if (d[i][j] == INF) {
        disconnected = true;
        break;
      }
      sum += d[i][j];
    }

    if (!disconnected && sum > 0) {
      values[i] = (double)(size - 1) / sum;
      sums[i] = sum;
      if (values[i] > maxVal)
        maxVal = values[i];
    } else {
      values[i] = -1.0; // Mark as disconnected
    }
  }

  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
    if (values[i] < 0) {
      fout << "x" << endl;
    } else {
      fout << (size - 1) << "/" << sums[i];
      // Identify most central node
      if (std::abs(values[i] - maxVal) < 1e-9) {
        fout << " <- Most Central";
      }
      fout << endl;
    }
  }

  fout.close();
  return true;
}