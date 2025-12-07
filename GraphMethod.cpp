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

#define INF 100000000

// Helper: build adjacency list (directed or undirected)
vector<vector<pair<int, int>>> getAdjacencyList(Graph *graph, char option) {
  int size = graph->getSize();
  vector<vector<pair<int, int>>> adj(size);

  for (int i = 0; i < size; i++) {
    map<int, int> m;
    if (option == 'O')
      graph->getAdjacentEdgesDirect(i, &m); // outgoing only
    else
      graph->getAdjacentEdges(i, &m); // all connected edges

    for (auto iter = m.begin(); iter != m.end(); iter++)
      adj[i].push_back({iter->first, iter->second});

    sort(adj[i].begin(), adj[i].end()); // ensure stable order
  }
  return adj;
}

// BFS traversal
bool BFS(Graph *graph, char option, int vertex, ofstream *fout) {
  if (vertex < 0 || vertex >= graph->getSize())
    return false;

  *fout << "========BFS========" << endl;

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  vector<bool> visited(graph->getSize(), false);
  queue<int> q;

  if (option == 'O')
    *fout << "Directed Graph BFS" << endl;
  else
    *fout << "Undirected Graph BFS" << endl;

  *fout << "Start: " << vertex << endl;

  q.push(vertex);
  visited[vertex] = true;

  bool first = true;
  while (!q.empty()) {
    int curr = q.front();
    q.pop();

    if (!first)
      *fout << " -> ";
    *fout << curr;
    first = false;

    for (auto &edge : adj[curr]) {
      int next = edge.first;
      if (!visited[next]) {
        visited[next] = true;
        q.push(next);
      }
    }
  }

  *fout << endl;
  return true;
}

// DFS traversal
bool DFS(Graph *graph, char option, int vertex, ofstream *fout) {
  if (vertex < 0 || vertex >= graph->getSize())
    return false;

  *fout << "========DFS========" << endl;

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  vector<bool> visited(graph->getSize(), false);
  stack<int> s;

  if (option == 'O')
    *fout << "Directed Graph DFS" << endl;
  else
    *fout << "Undirected Graph DFS" << endl;

  *fout << "Start: " << vertex << endl;

  for (auto &vec : adj)
    sort(vec.rbegin(), vec.rend()); // reverse order for stack usage

  s.push(vertex);
  bool first = true;

  while (!s.empty()) {
    int curr = s.top();
    s.pop();

    if (visited[curr])
      continue;
    visited[curr] = true;

    if (!first)
      *fout << " -> ";
    *fout << curr;
    first = false;

    for (auto &edge : adj[curr]) {
      int next = edge.first;
      if (!visited[next])
        s.push(next);
    }
  }

  *fout << endl;
  return true;
}

// Edge structure for Kruskal
struct Edge {
  int u, v, weight;
  bool operator<(const Edge &other) const {
    if (weight != other.weight)
      return weight < other.weight;
    if (u != other.u)
      return u < other.u;
    return v < other.v;
  }
};

// Disjoint-set structure
struct DisjointSet {
  vector<int> parent;
  DisjointSet(int n) {
    parent.resize(n);
    for (int i = 0; i < n; i++)
      parent[i] = i;
  }
  int find(int u) {
    if (u == parent[u])
      return u;
    return parent[u] = find(parent[u]);
  }
  void merge(int u, int v) {
    u = find(u);
    v = find(v);
    if (u != v)
      parent[u] = v;
  }
};

// Kruskal MST
bool Kruskal(Graph *graph, ofstream *fout) {
  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, 'X');
  vector<Edge> edges;
  int size = graph->getSize();

  for (int u = 0; u < size; u++) {
    for (auto &p : adj[u]) {
      int v = p.first;
      int w = p.second;
      if (u < v)
        edges.push_back({u, v, w});
    }
  }

  sort(edges.begin(), edges.end());
  DisjointSet ds(size);

  vector<Edge> mst;
  int costSum = 0;

  for (auto &e : edges) {
    if (ds.find(e.u) != ds.find(e.v)) {
      ds.merge(e.u, e.v);
      mst.push_back(e);
      costSum += e.weight;
    }
  }

  if (size > 1 && mst.size() < size - 1)
    return false; // graph is not connected

  vector<vector<pair<int, int>>> mstAdj(size);
  for (auto &e : mst) {
    mstAdj[e.u].push_back({e.v, e.weight});
    mstAdj[e.v].push_back({e.u, e.weight});
  }

  for (int i = 0; i < size; i++) {
    sort(mstAdj[i].begin(), mstAdj[i].end());
    *fout << "[" << i << "] ";
    for (auto &p : mstAdj[i])
      *fout << p.first << "(" << p.second << ") ";
    *fout << endl;
  }

  *fout << "Cost: " << costSum << endl;
  return true;
}

// Dijkstra shortest path
bool Dijkstra(Graph *graph, char option, int vertex, ofstream *fout) {
  if (vertex < 0 || vertex >= graph->getSize())
    return false;

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  int size = graph->getSize();

  // reject graphs with negative weights
  for (int i = 0; i < size; i++)
    for (auto &p : adj[i])
      if (p.second < 0)
        return false;

  vector<int> dist(size, INF);
  vector<int> parent(size, -1);

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
      if (dist[curr] + weight < dist[next]) {
        dist[next] = dist[curr] + weight;
        parent[next] = curr;
        pq.push({dist[next], next});
      }
    }
  }

  if (option == 'O')
    *fout << "Directed Graph Dijkstra" << endl;
  else
    *fout << "Undirected Graph Dijkstra" << endl;

  *fout << "Start: " << vertex << endl;

  for (int i = 0; i < size; i++) {
    *fout << "[" << i << "] ";
    if (dist[i] == INF) {
      *fout << "x" << endl; // unreachable
    } else {
      vector<int> path;
      int curr = i;
      while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
      }
      for (int k = path.size() - 1; k >= 0; k--) {
        *fout << path[k];
        if (k > 0)
          *fout << " -> ";
      }
      *fout << " (" << dist[i] << ")" << endl;
    }
  }

  return true;
}

// Bellman-Ford shortest path
bool Bellmanford(Graph *graph, char option, int s_vertex, int e_vertex,
                 ofstream *fout) {
  int size = graph->getSize();
  if (s_vertex < 0 || s_vertex >= size || e_vertex < 0 || e_vertex >= size)
    return false;

  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  vector<int> dist(size, INF);
  vector<int> parent(size, -1);

  dist[s_vertex] = 0;

  // main relaxation loop
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

  // negative-cycle detection
  for (int u = 0; u < size; u++) {
    if (dist[u] == INF)
      continue;
    for (auto &edge : adj[u]) {
      int v = edge.first;
      int w = edge.second;
      if (dist[u] + w < dist[v])
        return false;
    }
  }

  if (option == 'O')
    *fout << "Directed Graph Bellman-Ford" << endl;
  else
    *fout << "Undirected Graph Bellman-Ford" << endl;

  if (dist[e_vertex] == INF) {
    *fout << "x" << endl; // unreachable
  } else {
    vector<int> path;
    int curr = e_vertex;
    while (curr != -1) {
      path.push_back(curr);
      curr = parent[curr];
    }
    for (int k = path.size() - 1; k >= 0; k--) {
      *fout << path[k];
      if (k > 0)
        *fout << " -> ";
    }
    *fout << endl;
    *fout << "Cost: " << dist[e_vertex] << endl;
  }

  return true;
}

// Floyd-Warshall all-pairs shortest path
bool FLOYD(Graph *graph, char option, ofstream *fout) {
  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, option);
  int size = graph->getSize();
  vector<vector<int>> d(size, vector<int>(size, INF));

  for (int i = 0; i < size; i++)
    d[i][i] = 0;

  for (int u = 0; u < size; u++)
    for (auto &edge : adj[u])
      d[u][edge.first] = min(d[u][edge.first], edge.second);

  for (int k = 0; k < size; k++)
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        if (d[i][k] != INF && d[k][j] != INF)
          d[i][j] = min(d[i][j], d[i][k] + d[k][j]);

  for (int i = 0; i < size; i++)
    if (d[i][i] < 0)
      return false; // negative cycle detected

  if (option == 'O')
    *fout << "Directed Graph Floyd" << endl;
  else
    *fout << "Undirected Graph Floyd" << endl;

  *fout << "\t";
  for (int i = 0; i < size; i++)
    *fout << "[" << i << "]\t";
  *fout << endl;

  for (int i = 0; i < size; i++) {
    *fout << "[" << i << "]\t";
    for (int j = 0; j < size; j++) {
      if (d[i][j] == INF)
        *fout << "x\t";
      else
        *fout << d[i][j] << "\t";
    }
    *fout << endl;
  }

  return true;
}

// Closeness Centrality
bool Centrality(Graph *graph, ofstream *fout) {
  vector<vector<pair<int, int>>> adj = getAdjacencyList(graph, 'X');
  int size = graph->getSize();
  vector<vector<int>> d(size, vector<int>(size, INF));

  for (int i = 0; i < size; i++)
    d[i][i] = 0;

  for (int u = 0; u < size; u++)
    for (auto &edge : adj[u])
      d[u][edge.first] = min(d[u][edge.first], edge.second);

  for (int k = 0; k < size; k++)
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        if (d[i][k] != INF && d[k][j] != INF)
          d[i][j] = min(d[i][j], d[i][k] + d[k][j]);

  for (int i = 0; i < size; i++)
    if (d[i][i] < 0)
      return false; // negative cycle check

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
      values[i] = -1.0; // unreachable node
    }
  }

  for (int i = 0; i < size; i++) {
    *fout << "[" << i << "] ";
    if (values[i] < 0) {
      *fout << "x" << endl;
    } else {
      *fout << (size - 1) << "/" << sums[i];
      if (std::abs(values[i] - maxVal) < 1e-9)
        *fout << " <- Most Central";
      *fout << endl;
    }
  }

  return true;
}
