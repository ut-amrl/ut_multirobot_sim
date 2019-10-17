#ifndef DEVILLSROOM_GRAPH_MAXIMALCLIQUEGENERATOR
#define DEVILLSROOM_GRAPH_MAXIMALCLIQUEGENERATOR

#include<vector>
#include<typeinfo>

#include "graph.hpp"

namespace c0dex
{
    namespace graph
    {
        class MaximalCliqueGenerator
        {
            public:
                class Clique
                {
                    public:
                        typedef std::vector<Graph::Node*>::const_iterator const_iterator;
                        typedef std::vector<Graph::Node*>::iterator iterator;

                        Clique(const Graph* graphArg) : graph(graphArg) {};

                        void addNode(Graph::Node& node) { nodes.push_back(&node); };

                        Clique intersectWithNodesNeighbours(const Graph::Node& rhs) const;
                        bool isMaximalWhenExtended(const Graph::Node& current) const;
                        bool cliqueExtendableByNode(const Graph::Node& node) const;
                        bool isGeneratedFromTheFirstSourceClique(const Clique& srcclique, const Graph::Node& current) const;

                        size_t size() const { return nodes.size(); }
                        const_iterator begin() const { return nodes.begin(); }
                        const_iterator end() const { return nodes.end(); }
                        iterator begin() { return nodes.begin(); }
                        iterator end() { return nodes.end(); }

                        bool operator<(const Clique& rhs) const;

                    private:
                        class NeighboursNotInCliqueIterator : private boost::noncopyable
                        {
                            public:
                                NeighboursNotInCliqueIterator(const Clique& cliqueArg, const Graph::Node& node);
                                void next() { ++neighbourIt; findNext(); };
                                bool hasMore() const { return neighbourIt != node.end(); };
                                Graph::Node* operator*() { return *neighbourIt; };

                            private:
                                const Clique& clique;
                                const Graph::Node& node;
                                Clique::const_iterator memberIt;
                                Graph::Node::const_iterator neighbourIt;

                                void findNext();
                        };

                        iterator insertNodeIfNotAtLocation(Graph::Node& node, iterator insertLoc);
                        bool nodeWasInSourceClique(
                                const_iterator& sourceCliqueMemberIt,
                                const Clique& srcclique,
                                Graph::const_iterator& nodeIt) const;

                        std::vector<Graph::Node*> nodes;
                        const Graph* graph;
                };

                class Reporter {
                    public:
                        virtual void report(unsigned int nodes, unsigned int graphSize, unsigned int cliques) = 0;
                        virtual ~Reporter() {}
                };

                MaximalCliqueGenerator(const Graph& graphArg) : graph(graphArg), reporter(NULL) {};
                std::vector<Clique> generate() const;

                void setReporter(Reporter& reporterArg) { reporter = &reporterArg; }

            private:
                void report(unsigned int nodes, unsigned int graphSize, unsigned int cliques) const { if(NULL != reporter) reporter->report(nodes, graphSize, cliques); }

                Clique makeClique() const { return Clique(&graph); }
                Clique makeClique(Graph::Node& node) const
                {
                    Clique clique = makeClique();
                    clique.addNode(node);
                    return clique;
                }

                const Graph& graph;
                Reporter* reporter;
        };

        std::ostream& operator<< (std::ostream& os, std::vector<MaximalCliqueGenerator::Clique> cliques);
        std::ostream& operator<< (std::ostream& os, MaximalCliqueGenerator::Clique clique);
    }
}

#endif
