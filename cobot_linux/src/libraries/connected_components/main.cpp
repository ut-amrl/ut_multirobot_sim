#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

#include "connected_components.h"

using connected_components::compute_scc;
using connected_components::get_largest_components;
using std::vector;
using std::map;
using std::list;
using std::ifstream;
using std::cout;
using std::endl;

/**
 * Get the count of nodes from a graph file representation
 */
long get_node_count(const char filename[]) {
  // Open file and keep track of how many times the value changes
  ifstream graphFile(filename);
  long maxNodeIndex = 0;
  long nodeIndex = 0;

  while (graphFile) {
    // Check the node index
    graphFile >> nodeIndex;
    if (nodeIndex > maxNodeIndex) {
      maxNodeIndex = nodeIndex;
    }
    // Check the outgoing edge
    graphFile >> nodeIndex;
    if (nodeIndex > maxNodeIndex) {
      maxNodeIndex = nodeIndex;
    }
  }
  return maxNodeIndex;
}

/**
 * Parse an input file as a graph, and return the graph.
 */
vector< vector<long> > parse_file(const char filename[]) {
  // Get the node count and prepare the graph
  long nodeCount = get_node_count(filename);
  vector< vector<long> > graph(nodeCount);

  // Open file and extract the data
  ifstream graphFile(filename);

  long nodeIndex;
  long outIndex;

  while (graphFile) {
    graphFile >> nodeIndex;
    graphFile >> outIndex;

    // Add the new outgoing edge to the node
    graph[nodeIndex - 1].push_back(outIndex - 1);
  }

  return graph;
}

int main() {
  const char FILENAME[] = "SCC.txt";
  // Get the sequential graph representation from the file
  vector< vector<long> > graph = parse_file(FILENAME);

  // Compute the strongly-connected components
  map< long, vector<long> > scc = compute_scc(graph);

  // Compute the largest 5 components and print them out
  list<unsigned long> largestComponents = get_largest_components(scc, 5);

  list<unsigned long>::iterator it;
  for (it = largestComponents.begin(); it != largestComponents.end(); it++) {
    cout << *it << ' ';
  }
  cout << endl;

  return 0;
}