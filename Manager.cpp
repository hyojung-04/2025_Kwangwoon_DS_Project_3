#include "Manager.h"
#include "GraphMethod.h"
#include "ListGraph.h"
#include "MatrixGraph.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

Manager::Manager() {
  graph = nullptr;
  fout.open("log.txt", ios::app); // Open log file
  load = 0;
}

Manager::~Manager() {
  if (load)
    delete graph; // Free memory if graph exists
  if (fout.is_open())
    fout.close(); // Close log file
}

void Manager::run(const char *command_txt) {
  ifstream fin;
  fin.open(command_txt, ios_base::in); // Open command file

  if (!fin) {
    fout << "command file open error" << endl;
    return;
  }

  string line;
  while (getline(fin, line)) { // Read file line by line
    stringstream ss(line);
    string cmd;
    ss >> cmd; // Parse command

    if (cmd == "LOAD") {
      string filename;
      ss >> filename;
      if (LOAD(filename.c_str())) {
        fout << "========LOAD========" << endl;
        fout << "Success" << endl;
        fout << "====================" << endl << endl;
      } else {
        printErrorCode(100); // Load failed
      }
    } else if (cmd == "PRINT") {
      if (!PRINT())
        printErrorCode(200);
    } else if (cmd == "BFS") {
      fout << "bfs" << endl;
      char option;
      int vertex;
      ss >> option >> vertex;
      if (!mBFS(option, vertex))
        printErrorCode(300);
    } else if (cmd == "DFS") {
      char option;
      int vertex;
      ss >> option >> vertex;
      if (!mDFS(option, vertex))
        printErrorCode(400);
    } else if (cmd == "KRUSKAL") {
      if (!mKRUSKAL())
        printErrorCode(500);
    } else if (cmd == "DIJKSTRA") {
      char option;
      int vertex;
      ss >> option >> vertex;
      if (!mDIJKSTRA(option, vertex))
        printErrorCode(600);
    } else if (cmd == "BELLMANFORD") {
      char option;
      int s_vertex, e_vertex;
      ss >> option >> s_vertex >> e_vertex;
      if (!mBELLMANFORD(option, s_vertex, e_vertex))
        printErrorCode(700);
    } else if (cmd == "FLOYD") {
      char option;
      ss >> option;
      if (!mFLOYD(option))
        printErrorCode(800);
    } else if (cmd == "CENTRALITY") {
      if (!mCentrality())
        printErrorCode(900);
    } else if (cmd == "EXIT") {
      fout << "========EXIT========" << endl;
      fout << "Success" << endl;
      fout << "====================" << endl << endl;
      break; // Terminate program
    }
  }

  fin.close();
}

bool Manager::LOAD(const char *filename) {
  // 1. Delete existing graph
  if (graph != nullptr) {
    delete graph;
    graph = nullptr;
    load = 0;
  }

  ifstream file(filename);
  if (!file.is_open()) {
    return false; // File open failed
  }

  char type;
  int size;

  // Read graph type and size
  if (!(file >> type >> size)) {
    file.close();
    return false;
  }

  if (type == 'L') {
    graph = new ListGraph(type, size); // Create ListGraph

    string line;
    getline(file, line); // Skip newline after size

    int currentSrc = -1; // Current source vertex

    // Read line by line
    while (getline(file, line)) {
      if (line.empty())
        continue;

      stringstream ss(line);
      int temp;
      vector<int> nums;

      // Parse numbers from the line
      while (ss >> temp) {
        nums.push_back(temp);
      }

      if (nums.size() == 1) {
        // Single number indicates a new source vertex
        currentSrc = nums[0];
      } else if (nums.size() == 2) {
        // Two numbers indicate destination and weight
        if (currentSrc != -1) {
          // **FIXED: Changed insert to insertEdge**
          graph->insertEdge(currentSrc, nums[0], nums[1]);
        }
      }
    }

  } else if (type == 'M') {
    graph = new MatrixGraph(type, size); // Create MatrixGraph
    int weight;

    // Read matrix data
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (!(file >> weight))
          break;

        if (weight != 0) { // Add edge if weight is not 0
          // **FIXED: Changed insert to insertEdge**
          graph->insertEdge(i, j, weight);
        }
      }
    }
  } else {
    file.close();
    return false; // Invalid graph type
  }

  file.close();
  load = 1; // Load successful
  return true;
}

bool Manager::PRINT() {
  if (graph == nullptr)
    return false; // Graph not loaded

  // **FIXED: Changed print to printGraph and passed address of fout**
  // MatrixGraph implementation already prints header/footer, so we just call
  // it.
  if (graph->printGraph(&fout)) {
    return true;
  }
  return false;
}

bool Manager::mBFS(char option, int vertex) {
  if (graph == nullptr)
    return false;

  fout << "========BFS========" << endl;
  // Call BFS algorithm
  if (BFS(graph, option, vertex)) {
    fout << "===================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mDFS(char option, int vertex) {
  if (graph == nullptr)
    return false;

  fout << "========DFS========" << endl;
  // Call DFS algorithm
  if (DFS(graph, option, vertex)) {
    fout << "===================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mDIJKSTRA(char option, int vertex) {
  if (graph == nullptr)
    return false;

  fout << "========DIJKSTRA========" << endl;
  // Call Dijkstra algorithm
  if (Dijkstra(graph, option, vertex)) {
    fout << "========================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mKRUSKAL() {
  if (graph == nullptr)
    return false;

  fout << "========KRUSKAL========" << endl;
  // Call Kruskal algorithm
  if (Kruskal(graph)) {
    fout << "=======================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) {
  if (graph == nullptr)
    return false;

  fout << "========BELLMANFORD========" << endl;
  // Call Bellman-Ford algorithm
  if (Bellmanford(graph, option, s_vertex, e_vertex)) {
    fout << "===========================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mFLOYD(char option) {
  if (graph == nullptr)
    return false;

  fout << "========FLOYD========" << endl;
  // Call Floyd algorithm
  if (FLOYD(graph, option)) {
    fout << "=====================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mCentrality() {
  if (graph == nullptr)
    return false;

  fout << "========CENTRALITY========" << endl;
  // Call Centrality algorithm
  if (Centrality(graph)) {
    fout << "==========================" << endl << endl;
    return true;
  }
  return false;
}

void Manager::printErrorCode(int n) {
  fout << "========ERROR========" << endl;
  fout << n << endl; // Print error code
  fout << "=====================" << endl << endl;
}