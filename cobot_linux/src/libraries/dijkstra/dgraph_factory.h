#ifndef DGRAPH_FACTORY_H
#define DGRAPH_FACTORY_H
/* Directed Graph Factory (produces various kinds of directed graphs)
 * ----------------------------------------------------------------------------
 * Author:  Shane Saunders
 */

/* default edge cost range */
const int DGRAPH_MIN_EDGE_COST = 1;
const int DGRAPH_MAX_EDGE_COST = 1;

class DGraph;  // forward declaration of graph class

/*--- Directed Graph Factory Class ------------------------------------------*/
 
/* This implements a singleton DGraphFactory object which contains member
 * functions for creating different types of directed graphs.  The single
 * DGraphFactory object can be accessed through the pointer returned by the
 * instancePtr() member function.
 *
 * The following member functions create different types of graphs, and each
 * return a pointer to the constructed graph.
 *
 * - newRandomSparse()
 * Creates a directed random graph with all vertices reachable from the
 * starting vertex (vertex 0).  Parameter n is the number of vertices in the
 * graph, and parameter edgeFactor is the average number of outgoing edges per
 * vertex.
 *
 * - newRandomDense()
 * Creates a directed random graph with all vertices reachable from the
 * starting vertex.  Parameter n is the size of the graph, and parameter
 * prob is the probability of edge existence.
 */
class DGraphFactory {
  public:
    static DGraphFactory *instancePtr() {
         static DGraphFactory instance;
         return &instance;
    }
    void setEdgeCostRange(int min, int max) {
        edgeCostMin = min; edgeCostMax = max; edgeCostR = 1 + max - min;
    };

    DGraph *newRandomSparse(int n, double edgeFactor,
        bool spanFromSource = true) const;
    DGraph *newRandomDense(int n, double prob,
        bool spanFromSource = true) const;
    
  protected:  // constructor for DGraphFactory objects is hidden
    DGraphFactory() {
        setEdgeCostRange(DGRAPH_MIN_EDGE_COST, DGRAPH_MAX_EDGE_COST); };
    DGraphFactory(const DGraphFactory&);  // hide copy constructor
  
  private:
    inline void addRandomEdgeCost(DGraph *g, int sourceNo, int destNo) const;
    void addSpanningTree(DGraph *g) const;
    void addSpanningCycle(DGraph *g) const;    
    
    int edgeCostMin, edgeCostMax, edgeCostR;
};

const int SourceVertex = 0;

#endif
