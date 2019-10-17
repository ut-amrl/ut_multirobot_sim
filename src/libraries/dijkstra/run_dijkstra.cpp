#include <iostream>
#include "dgraph.h"
#include "dgraph_factory.h"  // random graph generation
#include "dijkstra.h"   // Dijkstra's algorithm
#include "timer.h"      // timer

using namespace std;

int main(void)
{
  cout << "Enter maximum edge cost: ";
  int maxEdgeCost;
  cin >> maxEdgeCost;

  cout << "Enter num vertices: ";
  int numVertices;
  cin >> numVertices;
  // Create a random graph of numVertices vertices with an average of 2.5
  // outgoing edges per vertex.
  DGraphFactory *f = DGraphFactory::instancePtr();
  f->setEdgeCostRange(1,maxEdgeCost);
  DGraph *g = f->newRandomSparse(numVertices,8.0);

  // Create an instance of Dijkstra's algorithm for use with up to numVertices
  // vertices.
  Dijkstra *dijkstra = new Dijkstra(numVertices);

  // time a run of Dijkstra's algorithm on the graph
  long d[numVertices];  // result array
  for(int v = 0; v < numVertices; v++) d[v] = INFINITE_DIST;  // initialise
  double t1 = GetTimeSec();
  // specify the graph
  dijkstra->init(g);
  // run: this assumes that vertex 0 is the starting vertex
  dijkstra->run(d, 0, rand() % numVertices);
  double t2 = GetTimeSec();
  cout << "Time = " << 1000.0 * (t2 - t1) << " msec" << endl;

  // tidy up
  delete dijkstra;
  delete g;
  //delete f;
}
