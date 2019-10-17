#include <iostream>
#include "graph.hpp"
#include "maximalcliquegenerator.h"


void loadFileToBuilder(char fileName[], c0dex::graph::Graph::Builder& builder);

class Progress : public c0dex::graph::MaximalCliqueGenerator::Reporter
{
    public:
        Progress() : lastCliqueNumber(0) {}

        virtual void report(unsigned int nodes, unsigned int graphSize, unsigned int cliques)
        {
            if(lastCliqueNumber + 100 <= cliques || nodes == graphSize)
            {
                std::cerr << "Processed nodes: " << nodes << "/" << graphSize << " Detected maximal cliques: " << cliques << "\r";
                lastCliqueNumber = cliques;
                if(nodes == graphSize) std::cerr << std::endl;
            }
        }

    private:
        unsigned int lastCliqueNumber;
};
