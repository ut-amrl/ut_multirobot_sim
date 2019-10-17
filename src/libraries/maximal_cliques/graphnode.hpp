#ifndef DEVILLSROOM_GRAPH_GRAPHNODE
#define DEVILLSROOM_GRAPH_GRAPHNODE

#include <string>

#include <boost/utility.hpp>

namespace c0dex
{
    namespace graph
    {
        class GraphNode : private boost::noncopyable
        {
            public:
                typedef std::vector<GraphNode*> neighbours_t;
                typedef neighbours_t::const_iterator const_iterator;

                GraphNode(std::string nameArg, int positionArg) : name(nameArg), position(positionArg) {};
                void addNeighbour(GraphNode& otherNode) { neighbours.push_back(&otherNode); };

                std::string getName() const { return name; }
                int getPosition() const { return position; }

                const_iterator begin() const { return neighbours.begin(); }
                const_iterator end() const { return neighbours.end(); }

            private:
                std::string name;
                neighbours_t neighbours;
                int position;
        };
    }
}

#endif
