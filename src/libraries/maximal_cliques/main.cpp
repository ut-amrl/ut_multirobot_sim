#include<iostream>
#include<fstream>
#include<algorithm>
#include<vector>


#include "main.h"

using namespace std;
using namespace c0dex::graph;

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        cout << "Please specify a filename" << endl;
        return 1;
    }

    char* fileName = argv[1];
    
    Graph::Builder builder;
    loadFileToBuilder(fileName, builder);

    Graph graph(builder);
    builder.reset();

    MaximalCliqueGenerator generator(graph);
    Progress progress;
    generator.setReporter(progress);

    vector<MaximalCliqueGenerator::Clique> maximalCliques = generator.generate();

    sort(maximalCliques.begin(),maximalCliques.end());

    cout << maximalCliques.size() << " cliques found.\n";
}

void loadFileToBuilder(char fileName[], Graph::Builder& builder)
{
    ifstream fp(fileName);

    string date;
    string baseId;
    string targetId;

    while(true)
    {
        getline(fp, date, '\t');
        getline(fp, baseId, '\t');
        getline(fp, targetId, '\n');

        if(!fp.good()) return;

        builder.addEdge(baseId,targetId);
    }
}
