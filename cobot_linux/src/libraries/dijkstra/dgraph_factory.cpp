#include <cstdlib>
#include "dgraph_factory.h"
#include "dgraph.h"
/* Directed Graph Factory (produces various kinds of directed graphs)
 * ----------------------------------------------------------------------------
 * Author:  Shane Saunders
 */
 
#define UNDEFINED -1

/*--- DGraphFactory ---------------------------------------------------------*/

/* --- uRand ---
 */
inline double uRand() {
    return rand() / RAND_MAX;
}

inline void DGraphFactory::addRandomEdgeCost(
    DGraph *g, int sourceNo, int destNo) const
{
    g->addNewEdge(sourceNo, destNo, edgeCostMin + (rand() % edgeCostR));
}

void DGraphFactory::addSpanningTree(DGraph *g) const
{
    for(int v = 1; v < g->nVertices; v++) addRandomEdgeCost(g, rand() % v, v);
    addRandomEdgeCost(g, g->nVertices-1, 0);  // tree plus one edge to make up n
}

void DGraphFactory::addSpanningCycle(DGraph *g) const
{
    for(int v = 1; v < g->nVertices; v++) addRandomEdgeCost(g, v-1, v);
    addRandomEdgeCost(g, g->nVertices-1, 0);
}

/* --- newRandomSparse() ---
 */
DGraph *DGraphFactory::newRandomSparse(int n, double edgeFactor,
    bool spanFromSource) const
{
    int i, j;

    /* allocate space for the graph */
    DGraph *g = new DGraph(n);
    
    /* initialise edgeCount and calculate the number of edges required */
    int edgeCount = 0;
    const int nEdges = (int)(n * edgeFactor);

    /* add a random spanning tree if necessary */
    if(spanFromSource) {
        addSpanningCycle(g);
        edgeCount += n;
    }
    
    /* Create the random graph.  Edges from vertex i to vertex j.
     * Variable k keeps track of the number of vertices in the out set
     * of the current vertex.
     */
    while(edgeCount < nEdges) {

        /* select random i and j */
        i = rand() % n;  j = rand() % n;
        
        if(i != j && !g->edgeExists(i, j)) {
            addRandomEdgeCost(g, i, j);
            edgeCount++;
        }
    }

    return g;
}

/* --- newRandomDense() ---
 */
DGraph *DGraphFactory::newRandomDense(int n, double prob,
    bool spanFromSource) const
{

    /* allocate space for the graph */
    DGraph *g = new DGraph(n);

    int *sources = new int[n];
    for(int i = 0; i < n; i++) sources[i] = UNDEFINED;
    
    /* add a random spanning cycle */
    if(spanFromSource) {
        /* determine random spanning edges to be added */
        for(int v = 1; v < g->nVertices; v++) sources[v] = v-1;
        sources[0] = g->nVertices-1;
        
        /* The probability of edge must be changed since n edges have
         * already been determined.
         */
        int maxEdges = n*(n-1);
        prob = (prob*maxEdges - n) / (maxEdges - n);
    }
    
    /* Create the random graph.  Edges from vertex i to vertex j.
     * Variable k keeps track of the number of vertices in the out set
     * of the current vertex.
     */
    int i, j;
    for(i = 0; i < n; i++) {
        for(j = 0; j < n; j++) {

            /* only create edges that are not from a vertex to itself */
            if (i != j) {
                
                /* Create an edge from vertex i to vertex j with
                 * probability prob.  All edges assigned for the connecting
                 * sub-graph as identified in sources[] will be automatically
                 * created.
                 */
                if(uRand() <= prob || sources[j] == i) {
                    addRandomEdgeCost(g, i, j);
                }
            }
        }
    }

    /* finished using sources[] */
    delete [] sources;

    return g;
}

/*---------------------------------------------------------------------------*/
