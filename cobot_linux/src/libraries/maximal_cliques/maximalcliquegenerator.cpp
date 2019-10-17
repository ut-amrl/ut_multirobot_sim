#include<iterator>
#include<algorithm>
#include<iostream>

#include "maximalcliquegenerator.h"

using namespace std;

namespace c0dex
{
    namespace graph
    {
        vector<MaximalCliqueGenerator::Clique> MaximalCliqueGenerator::generate() const
        {
            vector<Clique> maximalCliques;
            Graph::const_iterator currentNodeIt = graph.begin();

            maximalCliques.push_back(makeClique(**currentNodeIt));

            unsigned int processedNodes = 1;
            
            while( (++currentNodeIt) != graph.end() )
            {
                for(int i = 0, nofOldCliques = maximalCliques.size(); i < nofOldCliques; ++i)
                {
                    Clique candidateClique = maximalCliques[i].intersectWithNodesNeighbours(**currentNodeIt);

                    if( candidateClique.size() == maximalCliques[i].size() )
                    {
                        maximalCliques[i].addNode(**currentNodeIt);
                    }
                    else if( candidateClique.isMaximalWhenExtended(**currentNodeIt) &&
                             candidateClique.isGeneratedFromTheFirstSourceClique(maximalCliques[i], **currentNodeIt) )
                    {
                        candidateClique.addNode(**currentNodeIt);
                        maximalCliques.push_back(candidateClique);
                    }
                }
                report(++processedNodes, graph.size(), maximalCliques.size());
            }

            return maximalCliques;
        }

        MaximalCliqueGenerator::Clique MaximalCliqueGenerator::Clique::intersectWithNodesNeighbours(const Graph::Node& node) const
        {
            Clique result(graph);

            const_iterator memberIt = begin();
            Graph::Node::const_iterator nodeIt = node.begin();

            while(memberIt != end() && nodeIt != node.end())
            {
                if( *memberIt == *nodeIt )
                {
                    result.addNode(**memberIt);
                    ++memberIt; ++nodeIt;
                }
                else if( (*memberIt)->getPosition() < (*nodeIt)->getPosition() )
                {
                    ++memberIt;
                }
                else
                {
                    ++nodeIt;
                }
            }
            return result;
        }

        bool MaximalCliqueGenerator::Clique::isMaximalWhenExtended(const Graph::Node& current ) const
        {
            for(NeighboursNotInCliqueIterator neighbourIt(*this, current); neighbourIt.hasMore(); neighbourIt.next())
                if(cliqueExtendableByNode(**neighbourIt))
                    return false;

            return true;
        }
        

        // Although this could be implemented as a variation on intersectWithNodesNeighbours
        // by returning early if we can, we gain a x2 speedup
        bool MaximalCliqueGenerator::Clique::cliqueExtendableByNode(const Graph::Node& node) const
        {
            const_iterator memberIt = begin();
            Graph::Node::const_iterator nodeIt = node.begin();

            while(memberIt != end() && nodeIt != node.end())
            {
                if( *memberIt == *nodeIt )
                {
                    ++memberIt; ++nodeIt;
                }
                else if( (*memberIt)->getPosition() < (*nodeIt)->getPosition() )
                {
                    return false;
                }
                else
                {
                    ++nodeIt;
                }
            }
            return memberIt == end();
        }
        
        bool MaximalCliqueGenerator::Clique::isGeneratedFromTheFirstSourceClique(
                const Clique& sourceClique, const Graph::Node& newNode) const
        {
            const_iterator sourceCliqueMemberIt = sourceClique.begin();
            Clique testClique = *this;
            iterator insertLocation = testClique.begin();
            Graph::const_iterator nodeIt;

            for (nodeIt = graph->begin(); *nodeIt != &newNode; ++nodeIt)
            {
                if (nodeWasInSourceClique(sourceCliqueMemberIt, sourceClique, nodeIt))
                {
                    insertLocation = testClique.insertNodeIfNotAtLocation(**sourceCliqueMemberIt, insertLocation);
                    ++insertLocation;
                    ++sourceCliqueMemberIt;
                }
                else
                {
                    if(testClique.cliqueExtendableByNode(**nodeIt))
                        return false;
                }
            }
            return true;
        }

        MaximalCliqueGenerator::Clique::iterator MaximalCliqueGenerator::Clique::insertNodeIfNotAtLocation(Graph::Node& node, iterator insertLoc)
        {
            if( insertLoc == end() || *insertLoc != &node )
                insertLoc = nodes.insert( insertLoc, &node );
            return insertLoc;
        }

        bool MaximalCliqueGenerator::Clique::nodeWasInSourceClique(
                const_iterator& sourceCliqueMemberIt,
                const Clique& srcclique,
                Graph::const_iterator& nodeIt) const
        {
            return sourceCliqueMemberIt != srcclique.end() && *sourceCliqueMemberIt == *nodeIt;
        }


        bool MaximalCliqueGenerator::Clique::operator<(const MaximalCliqueGenerator::Clique& rhs) const
        {
            MaximalCliqueGenerator::Clique::const_iterator it;
            MaximalCliqueGenerator::Clique::const_iterator jt;

            for( it = begin(), jt = rhs.begin(); it != end() && jt!=rhs.end(); ++it, ++jt )
                if((*it)->getPosition() < (*jt)->getPosition())
                    return true;
                else if((*it)->getPosition() > (*jt)->getPosition())
                    return false;

            return it == end() && jt != rhs.end();
        }

        MaximalCliqueGenerator::Clique::NeighboursNotInCliqueIterator::NeighboursNotInCliqueIterator(const Clique& cliqueArg, const Graph::Node& nodeArg)
            : clique(cliqueArg)
            , node(nodeArg)
        {
            memberIt = clique.begin();
            neighbourIt = node.begin();
            findNext();
        }

        void MaximalCliqueGenerator::Clique::NeighboursNotInCliqueIterator::findNext()
        {
            while( neighbourIt != node.end() && memberIt != clique.end())
            {
                if( (*memberIt)->getPosition() == (*neighbourIt)->getPosition() )
                {
                    ++memberIt; ++neighbourIt;
                }
                else if( (*memberIt)->getPosition() < (*neighbourIt)->getPosition() )
                {
                    ++memberIt;
                }
                else
                {
                    return;
                }
            }
            if( neighbourIt != node.end() && (*neighbourIt)->getPosition() > node.getPosition() )
                neighbourIt = node.end();
        }

        ostream& operator<< (ostream& os, std::vector<MaximalCliqueGenerator::Clique> cliques)
        {
            vector<MaximalCliqueGenerator::Clique>::iterator it;
            for( it = cliques.begin(); it != cliques.end(); ++it )
                if( it->size() >2 )
                    cout << *it << endl;

            return os;
        }

        ostream& operator<< (ostream& os, MaximalCliqueGenerator::Clique clique)
        {
            if(clique.size()>0)
            {
                MaximalCliqueGenerator::Clique::const_iterator jt = clique.begin();
                os << (*jt)->getName();
                for( ++jt; jt != clique.end(); ++jt )
                    os << ", " << (*jt)->getName();
            }
            return os;
        }
    }
}
