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
  fout.open("log.txt", ios::out); // create log file
  load = 0;
}

Manager::~Manager() {
  if (load)
    delete graph; // free graph memory
  if (fout.is_open())
    fout.close(); // close log file
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

    string dump; // Variable to check for extra arguments

    if (cmd == "LOAD") {
      string filename;
      // Check if there is exactly 1 argument (filename) and no extra args
      if (ss >> filename && !(ss >> dump)) {
        if (LOAD(filename.c_str())) {
          fout << "========LOAD========" << endl;
          fout << "Success" << endl;
          fout << "====================" << endl << endl;
        } else {
          printErrorCode(100); // Load failed
        }
      } else {
        printErrorCode(100); // Error: Invalid argument count
      }

    } else if (cmd == "PRINT") {
      // Check if there are no arguments
      if (!(ss >> dump)) {
        if (!PRINT())
          printErrorCode(200);
      } else {
        printErrorCode(200);
      }

    } else if (cmd == "BFS") {
      char option;
      int vertex;
      // Check if there are exactly 2 arguments
      if (ss >> option >> vertex && !(ss >> dump)) {
        if (!mBFS(option, vertex))
          printErrorCode(300);
      } else {
        printErrorCode(300);
      }

    } else if (cmd == "DFS") {
      char option;
      int vertex;
      // Check if there are exactly 2 arguments
      if (ss >> option >> vertex && !(ss >> dump)) {
        if (!mDFS(option, vertex))
          printErrorCode(400);
      } else {
        printErrorCode(400);
      }

    } else if (cmd == "KRUSKAL") {
      // Check if there are no arguments
      if (!(ss >> dump)) {
        if (!mKRUSKAL())
          printErrorCode(500);
      } else {
        printErrorCode(500);
      }

    } else if (cmd == "DIJKSTRA") {
      char option;
      int vertex;
      // Check if there are exactly 2 arguments
      if (ss >> option >> vertex && !(ss >> dump)) {
        if (!mDIJKSTRA(option, vertex))
          printErrorCode(600);
      } else {
        printErrorCode(600);
      }

    } else if (cmd == "BELLMANFORD") {
      char option;
      int s_vertex, e_vertex;
      // Check if there are exactly 3 arguments
      if (ss >> option >> s_vertex >> e_vertex && !(ss >> dump)) {
        if (!mBELLMANFORD(option, s_vertex, e_vertex))
          printErrorCode(700);
      } else {
        printErrorCode(700);
      }

    } else if (cmd == "FLOYD") {
      char option;
      // Check if there is exactly 1 argument
      if (ss >> option && !(ss >> dump)) {
        if (!mFLOYD(option))
          printErrorCode(800);
      } else {
        printErrorCode(800);
      }

    } else if (cmd == "CENTRALITY") {
      // Check if there are no arguments
      if (!(ss >> dump)) {
        if (!mCentrality())
          printErrorCode(900);
      } else {
        printErrorCode(900);
      }

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
  // delete previous graph
  if (graph != nullptr) {
    delete graph;
    graph = nullptr;
    load = 0;
  }

  ifstream file(filename);
  if (!file.is_open())
    return false;

  char type;
  int size;

  // read graph type and size
  if (!(file >> type >> size)) {
    file.close();
    return false;
  }

  // load list-based graph
  if (type == 'L') {
    graph = new ListGraph(type, size);

    string line;
    getline(file, line); // skip newline
    int currentSrc = -1;

    // read adjacency list format
    while (getline(file, line)) {
      if (line.empty())
        continue;

      stringstream ss(line);
      int temp;
      vector<int> nums;

      while (ss >> temp)
        nums.push_back(temp);

      if (nums.size() == 1) {
        // new source vertex
        currentSrc = nums[0];
      } else if (nums.size() == 2) {
        // destination + weight
        if (currentSrc != -1) {
          graph->insertEdge(currentSrc, nums[0], nums[1]);
        }
      }
    }

    // load matrix-based graph
  } else if (type == 'M') {
    graph = new MatrixGraph(type, size);
    int weight;

    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (!(file >> weight))
          break;

        if (weight != 0) {
          graph->insertEdge(i, j, weight);
        }
      }
    }

  } else {
    file.close();
    return false; // invalid graph type
  }

  file.close();
  load = 1;
  return true;
}

bool Manager::PRINT() {
  if (graph == nullptr)
    return false;

  // print using each graph's own print function
  if (graph->printGraph(&fout))
    return true;

  return false;
}

bool Manager::mBFS(char option, int vertex) {
  if (graph == nullptr)
    return false;

  // Pass &fout
  if (BFS(graph, option, vertex, &fout)) {
    fout << "===================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mDFS(char option, int vertex) {
  if (graph == nullptr)
    return false;

  // Pass &fout
  if (DFS(graph, option, vertex, &fout)) {
    fout << "===================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mDIJKSTRA(char option, int vertex) {
  if (graph == nullptr)
    return false;
  fout << "========DIJKSTRA========" << endl;

  // Pass &fout
  if (Dijkstra(graph, option, vertex, &fout)) {
    fout << "========================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mKRUSKAL() {
  if (graph == nullptr)
    return false;
  fout << "========KRUSKAL========" << endl;

  // Pass &fout
  if (Kruskal(graph, &fout)) {
    fout << "=======================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) {
  if (graph == nullptr)
    return false;
  fout << "========BELLMANFORD========" << endl;

  // Pass &fout
  if (Bellmanford(graph, option, s_vertex, e_vertex, &fout)) {
    fout << "===========================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mFLOYD(char option) {
  if (graph == nullptr)
    return false;
  fout << "========FLOYD========" << endl;

  // Pass &fout
  if (FLOYD(graph, option, &fout)) {
    fout << "=====================" << endl << endl;
    return true;
  }
  return false;
}

bool Manager::mCentrality() {
  if (graph == nullptr)
    return false;
  fout << "========CENTRALITY========" << endl;

  // Pass &fout
  if (Centrality(graph, &fout)) {
    fout << "==========================" << endl << endl;
    return true;
  }
  return false;
}

void Manager::printErrorCode(int n) {
  fout << "========ERROR========\n";
  fout << n << "\n";
  fout << "=====================\n\n";
}
