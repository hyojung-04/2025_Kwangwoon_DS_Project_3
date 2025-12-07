#ifndef _GRAPHMETHOD_H_
#define _GRAPHMETHOD_H_

#include "ListGraph.h"
#include "MatrixGraph.h"

bool BFS(Graph *graph, char option, int vertex, std::ofstream *fout);
bool DFS(Graph *graph, char option, int vertex, std::ofstream *fout);
bool Kruskal(Graph *graph, std::ofstream *fout);
bool Dijkstra(Graph *graph, char option, int vertex, std::ofstream *fout);
bool Bellmanford(Graph *graph, char option, int s_vertex, int e_vertex,
                 std::ofstream *fout);
bool FLOYD(Graph *graph, char option, std::ofstream *fout);
bool Centrality(Graph *graph, std::ofstream *fout);

#endif