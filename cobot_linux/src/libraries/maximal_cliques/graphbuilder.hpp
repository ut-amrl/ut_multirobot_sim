#ifndef DEVILLSROOM_GRAPH_GRAPHBUILDER
#define DEVILLSROOM_GRAPH_GRAPHBUILDER

#include <map>
#include <string>
#include <boost/utility.hpp>

#include "graphbuildernode.hpp"

namespace c0dex
{
    namespace graph
    {
        class GraphBuilder : private boost::noncopyable
        {
            public:
                typedef GraphBuilderNode Node;
                typedef std::map<std::string, GraphBuilderNode*> nodeMap_t;
                typedef std::map<std::string, GraphBuilderNode*>::iterator iterator;
                typedef std::map<std::string, GraphBuilderNode*>::const_iterator const_iterator;

                ~GraphBuilder()
                {
                    reset();
                }

                void reset()
                {
                    for (iterator it = nodeMap.begin(); it != nodeMap.end(); ++it)
                        delete it->second;

                    nodeMap.clear();
                }

                GraphBuilderNode& ensureNode(std::string name)
                {
                    std::pair<iterator, bool> insertPair = nodeMap.insert(std::pair<std::string, GraphBuilderNode*>(name, NULL));

                    if(insertPair.second)
                        insertPair.first->second = new GraphBuilderNode(name);

                    return *insertPair.first->second;
                }

                void addEdge(GraphBuilderNode& fromNode, GraphBuilderNode& toNode)
                {
                    fromNode.addEdge(toNode);
                    toNode.addEdge(fromNode);
                };

                void addEdge(std::string fromNode, std::string toNode)
                {
                    addEdge(ensureNode(fromNode),ensureNode(toNode));
                };

                void setPositions()
                {
                    unsigned int i = 0;
                    for(iterator kt = nodeMap.begin(); kt != nodeMap.end(); ++kt, ++i)
                    {
                        kt->second->setPosition(i);
                    }
                }

                const_iterator begin() const { return nodeMap.begin(); }
                const_iterator end() const { return nodeMap.end(); }

                size_t size() const { return nodeMap.size(); }

            private:
                nodeMap_t nodeMap;
        };
    }
}

#endif
