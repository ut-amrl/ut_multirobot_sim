#ifndef DEVILLSROOM_GRAPH_GRAPHBUILDERNODE
#define DEVILLSROOM_GRAPH_GRAPHBUILDERNODE

#include <string>
#include <set>
#include <algorithm>

#include <boost/utility.hpp>

#include "utility.hpp"

namespace c0dex
{
    namespace graph
    {
        class GraphBuilderNode : private boost::noncopyable
        {
            public:
                typedef std::set<GraphBuilderNode*, lessPtr<GraphBuilderNode*> > neighbours_t;
                typedef neighbours_t::const_iterator const_iterator;
                typedef neighbours_t::iterator iterator;

                GraphBuilderNode(std::string nameArg) : name(nameArg) {};

                void addEdge(GraphBuilderNode& otherNode) { if(&otherNode!=this) neighbours.insert(&otherNode); };

                const std::string getName() const { return name; };
                void setPosition(int positionArg) { position = positionArg; }
                int getPosition() const { return position; }

                const_iterator begin() const { return neighbours.begin(); }
                const_iterator end() const { return neighbours.end(); }

                bool operator< (const GraphBuilderNode& rhs) { return (0>getName().compare(rhs.getName())); }
                bool operator> (const GraphBuilderNode& rhs) { return (0<getName().compare(rhs.getName())); }

            private:
                std::string name;
                unsigned int position;
                neighbours_t neighbours;
        };
        
    }
}
#endif
