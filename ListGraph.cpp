#include "ListGraph.h"
#include <iostream>
#include <utility>

using namespace std;

ListGraph::ListGraph(bool type, int size) : Graph(type, size) {
  // Allocate array of maps
  // m_List[i] stores edges starting from vertex i (Key: Destination, Value:
  // Weight)
  m_List = new map<int, int>[size];
}

ListGraph::~ListGraph() { delete[] m_List; }

// Definition of getAdjacentEdges (No Direction == Undirected)
void ListGraph::getAdjacentEdges(int vertex, map<int, int> *m) {
  // 1. Get outgoing edges
  getAdjacentEdgesDirect(vertex, m);

  // 2. Get incoming edges (Search all other lists to find edges pointing to
  // 'vertex') This is required for treating a directed graph as undirected
  // during traversal.
  for (int i = 0; i < getSize(); i++) {
    if (i == vertex)
      continue;

    auto it = m_List[i].find(vertex);
    if (it != m_List[i].end()) {
      // Found incoming edge, insert it (Key: Start node 'i', Value: Weight)
      m->insert(make_pair(i, it->second));
    }
  }
}

// Definition of getAdjacentEdgesDirect (Directed graph)
void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int> *m) {
  // Copy all outgoing edges from the adjacency list of 'vertex'
  for (auto it = m_List[vertex].begin(); it != m_List[vertex].end(); it++) {
    m->insert(make_pair(it->first, it->second));
  }
}

// Definition of insertEdge
void ListGraph::insertEdge(int from, int to, int weight) {
  if (from < 0 || from >= getSize() || to < 0 || to >= getSize())
    return;

  // Insert into map
  m_List[from][to] = weight;
}

// Definition of print Graph
bool ListGraph::printGraph(ofstream *fout) {
  if (getSize() < 0)
    return false;

  *fout << "========PRINT========" << endl;

  for (int i = 0; i < getSize(); i++) {
    *fout << "[" << i << "]";

    // Iterate through connected edges
    // map is automatically sorted by Key (Destination vertex), satisfying the
    // requirement.
    for (auto it = m_List[i].begin(); it != m_List[i].end(); it++) {
      *fout << " -> (" << it->first << "," << it->second << ")";
    }
    *fout << endl;
  }
  *fout << "=====================" << endl;
  return true;
}