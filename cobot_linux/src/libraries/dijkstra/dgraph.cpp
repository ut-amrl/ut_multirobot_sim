/* Directed Graphs
 * ----------------------------------------------------------------------------
 * Author:  Shane Saunders
 */
#include <cstdio>
#include "dgraph.h"

/*--- DGraph ----------------------------------------------------------------*/

DGraph::DGraph(int n)
{
  nVertices = n;
  vertices = new DGraphVertex[n];
  initVertices();
}

DGraph::~DGraph()
{
  clear();
  delete [] vertices;
}

void DGraph::clear()
{
  DGraphEdge *edge, *nextEdge;
  for(int i = 0; i < nVertices; i++) {
    edge = vertices[i].outHead;
    while(edge) {
      nextEdge = edge->nextOut;
      delete edge;
      edge = nextEdge;
    }
  }
  initVertices();
}

void DGraph::initVertices()
{
  for(int i = 0; i < nVertices; i++) {
    vertices[i].outHead = vertices[i].outTail = 0;
    vertices[i].inHead = vertices[i].inTail = 0;
    vertices[i].outSize = vertices[i].inSize = 0;
  }
}

void DGraph::addNewEdge(int source, int target, int dist)
{
  DGraphEdge *newEdge = new DGraphEdge;
  newEdge->source = source;
  newEdge->target = target;
  newEdge->dist = dist;
  newEdge->nextOut = NULL;
  newEdge->nextIn = NULL;
  newEdge->enabled = true;

  DGraphVertex *source_vertex = &vertices[source];
  if(source_vertex->outTail) {
    source_vertex->outTail->nextOut = newEdge;
  }
  else {
    source_vertex->outHead = newEdge;
  }
  source_vertex->outTail = newEdge;
  source_vertex->outSize++;

  DGraphVertex *target_vertex = &vertices[target];
  if(target_vertex->inTail) {
    target_vertex->inTail->nextIn = newEdge;
  }
  else {
    target_vertex->inHead = newEdge;
  }
  target_vertex->inTail = newEdge;
  target_vertex->inSize++;
};

void DGraph::modifyEdge(int source, int target, int dist, bool enabled)
{
  DGraphVertex *source_vertex = &vertices[source];
  DGraphEdge *edge = source_vertex->outHead;

  while(edge) {
    if(edge->target == target) {
      break;
    }
    edge = edge->nextOut;
  }
  if (edge) {
    edge->dist = dist;
    edge->enabled = enabled;
  } else {
    addNewEdge(source, target, dist);
    source_vertex->outTail->enabled = enabled;
  }
}

bool DGraph::edgeExists(int v, int w) const
{
  const DGraphEdge *edge = vertices[v].outHead;
  while(edge) {
    if(edge->target == w) return true;
    edge = edge->nextOut;
  }
  return false;
}

bool DGraph::reachable(int s) const
{
  int *stack = new int[nVertices];
  int tos = 0;

  int *visited = new int[nVertices];
  for(int i = 0; i < nVertices; i++) visited[i] = 0;

  int vertexCount = 0;
  visited[s] = 1;
  stack[tos++] = s;
  DGraphEdge *edge;
  int v, w;
  while(tos) {
    v = stack[--tos];
    vertexCount++;
    edge = vertices[v].outHead;
    while(edge) {
      w = edge->target;
      if(!visited[w]) {
        visited[w] = 1;
        stack[tos++] = w;
      }
      edge = edge->nextOut;
    }
  }

  delete [] stack;
  delete [] visited;

  return vertexCount == nVertices;
}

void DGraph::print() const
{
  const DGraphEdge *edge;

  printf("Graph (vertex: edge list) = \n");

  for(int i = 0; i < nVertices; i++) {
    printf("%d: ", i);
    edge = vertices[i].outHead;
    while(edge) {
      printf(" %d", edge->target);
      edge = edge->nextOut;
    }
    putchar('\n');
  }

  printf("Graph (vertex: edge{dist} list) = \n");

  for(int i = 0; i < nVertices; i++) {
    printf("%d: ", i);
    edge = vertices[i].outHead;
    while(edge) {
      printf(" %d{%d}", edge->target, edge->dist);
      edge = edge->nextOut;
    }
    putchar('\n');
  }
}
