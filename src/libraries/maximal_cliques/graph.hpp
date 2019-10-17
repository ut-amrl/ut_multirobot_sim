#ifndef DEVILLSROOM_GRAPH_GRAPH
#define DEVILLSROOM_GRAPH_GRAPH

#include <boost/utility.hpp>

#include "graphbuilder.hpp"
#include "graphnode.hpp"

namespace c0dex
{
    namespace graph
    {
        class Graph : private boost::noncopyable
        {
            public:
                typedef GraphBuilder Builder;
                typedef GraphNode Node;

                typedef std::vector<Node*> nodeVector_t;
                typedef std::vector<Node*>::iterator iterator;
                typedef std::vector<Node*>::const_iterator const_iterator;
                
                Graph(Builder& builder)
                {
                    builder.setPositions();
                    copyNodes(builder);
                    copyEdges(builder);
                }

                ~Graph() { reset(); }

                const_iterator begin() const { return nodes.begin(); }
                const_iterator end() const { return nodes.end(); }

                size_t size() const { return nodes.size(); }

            private:
                nodeVector_t nodes;

                void copyNodes(const Builder& builder)
                {
                    nodes.reserve(builder.size());
                    for (Builder::const_iterator kt = builder.begin(); kt != builder.end(); ++kt)
                        nodes.push_back(new Node(kt->first, kt->second->getPosition()));
                }

                void copyEdges(Builder& builder)
                {
                    int i = 0;
                    for (Builder::const_iterator kt = builder.begin(); kt != builder.end(); ++kt, ++i)
                        for (Builder::Node::const_iterator jt = kt->second->begin(); jt != kt->second->end(); jt++)
                            nodes[i]->addNeighbour(*nodes[(*jt)->getPosition()]);
                }

                void reset()
                {
                    for (iterator it = nodes.begin(); it != nodes.end(); ++it)
                        delete *it;

                    nodes.clear();
                }
        };
    }
}

#endif
