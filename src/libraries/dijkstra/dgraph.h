#ifndef DGRAPH_H
#define DGRAPH_H
/* Directed Graphs
 * ----------------------------------------------------------------------------
 * Author:  Shane Saunders
 */

/*--- Constants -------------------------------------------------------------*/

/* Infinite Distance */
const long INFINITE_DIST = 100000000;

/*--- Directed Graph Classes ------------------------------------------------*/

/* --- Directed graph edge class ---
 * Each edge object represents an outgoing edge of some vertex and is stored in
 * that vertexes linked list of edges.   The member 'target' is the edge's
 * target vertex number, whereas 'source' is the edge's source vertex number.
 * The member 'dist' is the associated edge distance.  The pointers 'nextIn'
 * and 'nextOut' are used to form a linked lists of a vertices incoming and
 * outgoing edges respectively.  Such linked lists are terminated with a null
 * pointer.
 */
class DGraphEdge {
  public:
    int source, target;
    int dist;
    DGraphEdge *nextOut, *nextIn;
    bool enabled;
};

/* --- Directed graph vertex class ---
 * Each vertex object has an associated linked lists of edge objects
 * representing the outgoing and incoming edges of that vertex.  The member
 * pointers outHead and inHead points to the first edge object in the linked
 * list of outgoing, and incoming edges respectively.  Similarly, outTail and
 * inTail point to the last edge of each linked list.  The number of outgoing
 * and incoming edges are stored in outSize and inSize respectively.
 */
class DGraphVertex {
  public:
    DGraphEdge *outHead, *outTail;
    DGraphEdge *inHead, *inTail;
    int outSize, inSize;
};

/* --- Directed graph class ---
 * Vertices in the graph are stored as an array of vertex objects, pointed to
 * by the member variable 'vertices'.  Each vertex is identified by a number
 * corresponding to its index in the vertices[] array.  The member
 * nVertices is the number of vertices in the graph.
 *
 * clear()      - Remove all edges from graph.
 *
 * addNewEdge() - Adds a new edge to the edge to the graph.
 *
 * print()      - Prints a text representation of the graph to the standard
 *                output.
 */
class DGraph {
 public:
  int nVertices;
  DGraphVertex *vertices;

  // Constructor: Creates a DGraph object containing n vertices.
  DGraph(int n);

  // Destructor.
  ~DGraph();

  // Clears all edges from the graph.
  void clear();

  // Adds a new edge from vertex 'source' to vertex 'target' with a
  // corresponding distance of dist.
  void addNewEdge(int srcVertexNo, int destVertexNo, int dist);

  // Modify an existing edge from vertex 'source' to vertex 'target', updating
  // its distance 'dist', and whether it is enabled or not. If an existing edge
  // is not found from vertex 'source' to vertex 'target', a new edge is added.
  void modifyEdge(int srcVertexNo, int destVertexNo, int dist, bool enabled);

  // Scan all existing edges from v to determine whether an edge to w exists.
  bool edgeExists(int v, int w) const;

  // Test whether all vertices are reachable from the source vertex s.
  bool reachable(int s) const;

  // Prints a text representation of the graph to the standard output.
  void print() const;

 private:

  // Initialize edge linked lists associated with every vertex.
  void initVertices();
};

/*---------------------------------------------------------------------------*/
#endif
