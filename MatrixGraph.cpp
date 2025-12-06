#include "MatrixGraph.h"
#include <cstring> // for memset
#include <iostream>
#include <string>
#include <vector>

using namespace std;

MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size) {
  // Dynamically allocate 2D array
  m_Mat = new int *[size];
  for (int i = 0; i < size; i++) {
    m_Mat[i] = new int[size];
    // Initialize to 0 (No connection)
    memset(m_Mat[i], 0, sizeof(int) * size);
  }
}

MatrixGraph::~MatrixGraph() {
  // Memory deallocation
  for (int i = 0; i < getSize(); i++) {
    delete[] m_Mat[i];
  }
  delete[] m_Mat;
}

void MatrixGraph::getAdjacentEdges(int vertex, map<int, int> *m) {
  // For Undirected graph traversal (BFS X, DFS X)
  // 1. Get outgoing edges (Row check)
  getAdjacentEdgesDirect(vertex, m);

  // 2. Get incoming edges (Column check)
  for (int i = 0; i < getSize(); i++) {
    if (m_Mat[i][vertex] != 0) {
      // Insert edge if it doesn't exist in the map
      m->insert(make_pair(i, m_Mat[i][vertex]));
    }
  }
}

void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int> *m) {
  // For Directed graph traversal (BFS O, DFS O)
  for (int i = 0; i < getSize(); i++) {
    if (m_Mat[vertex][i] != 0) {
      // Insert pair (destination, weight) to map
      m->insert(make_pair(i, m_Mat[vertex][i]));
    }
  }
}

void MatrixGraph::insertEdge(int from, int to, int weight) {
  // Validate indices
  if (from < 0 || from >= getSize() || to < 0 || to >= getSize())
    return;

  m_Mat[from][to] = weight;
}

bool MatrixGraph::printGraph(ofstream *fout) {
  if (getSize() < 0)
    return false;

  *fout << "========PRINT========" << endl;

  // Print Header Row
  *fout << "\t";
  for (int i = 0; i < getSize(); i++) {
    *fout << "[" << i << "]"
          << "\t";
  }
  *fout << endl;

  // Print Rows
  for (int i = 0; i < getSize(); i++) {
    *fout << "[" << i << "]"
          << "\t"; // Row Header
    for (int j = 0; j < getSize(); j++) {
      *fout << m_Mat[i][j] << "\t";
    }
    *fout << endl;
  }
  *fout << "=====================" << endl;
  return true;
}