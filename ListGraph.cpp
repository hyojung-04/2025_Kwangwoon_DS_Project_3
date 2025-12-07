#include "ListGraph.h"
#include <iostream>
#include <utility>

using namespace std;

ListGraph::ListGraph(bool type, int size) : Graph(type, size) {
  // create array of maps for adjacency list
  // each map holds: {to : weight}
  m_List = new map<int, int>[size];
}

ListGraph::~ListGraph() {
  // free adjacency list memory
  delete[] m_List;
}

// get edges for undirected view
void ListGraph::getAdjacentEdges(int vertex, map<int, int> *m) {
  // add outgoing edges
  getAdjacentEdgesDirect(vertex, m);

  // add incoming edges by scanning all lists
  for (int i = 0; i < getSize(); i++) {
    if (i == vertex)
      continue;

    auto it = m_List[i].find(vertex);
    if (it != m_List[i].end()) {
      // add edge pointing to this vertex
      m->insert(make_pair(i, it->second));
    }
  }
}

// get outgoing edges only (directed view)
void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int> *m) {
  // copy all outgoing edges of this vertex
  for (auto it = m_List[vertex].begin(); it != m_List[vertex].end(); it++) {
    m->insert(make_pair(it->first, it->second));
  }
}

// add new edge to adjacency list
void ListGraph::insertEdge(int from, int to, int weight) {
  // check range
  if (from < 0 || from >= getSize() || to < 0 || to >= getSize())
    return;

  // store edge in map
  m_List[from][to] = weight;
}

// print adjacency list
bool ListGraph::printGraph(ofstream *fout) {
  if (getSize() < 0)
    return false;

  *fout << "========PRINT========" << endl;

  for (int i = 0; i < getSize(); i++) {
    *fout << "[" << i << "]";

    if (m_List[i].empty()) {
      *fout << " ->";
    }

    // Iterate through connected edges
    // map is automatically sorted by Key (Destination vertex)
    for (auto it = m_List[i].begin(); it != m_List[i].end(); it++) {
      *fout << " -> (" << it->first << "," << it->second << ")";
    }
    *fout << endl;
  }
  *fout << "=====================" << endl;
  return true;
}
