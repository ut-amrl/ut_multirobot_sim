#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <list>

#ifndef CONNECTED_COMPONENTS_H_
#define CONNECTED_COMPONENTS_H_

namespace connected_components {

std::map<long, std::vector<long> > compute_scc(
    std::vector<std::vector<long> > &graph);

std::vector<std::vector<long> > reverse_graph(
    const std::vector<std::vector<long> > &graph);

void dfs_loop(const std::vector<std::vector<long> > &graph,
              std::vector<long> &finishTime,
              std::vector<long> &leader);

long dfs(const std::vector<std::vector<long> > &graph, long nodeIndex,
         std::vector<bool> &expanded, std::vector<long> &finishTime, long t,
         std::vector<long> &leader, long s);

std::list<unsigned long> get_largest_components(
    const std::map<long, std::vector<long> > scc, long size);


}  // namespace connected_components {

#endif  // CONNECTED_COMPONENTS_H_