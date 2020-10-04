#ifndef OSRM_EXTRACTOR_RESTRICTION_GRAPH_HPP_
#define OSRM_EXTRACTOR_RESTRICTION_GRAPH_HPP_

#include "extractor/restriction_filter.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
#include <boost/assert.hpp>
#include <boost/container_hash/hash.hpp>
#include <set>
#include <unordered_map>

namespace osrm
{
namespace extractor
{

struct RestrictionEdge {
    NodeID external_node;
    size_t restriction_way;
    bool is_transfer;
};

struct RestrictionWay {
    std::vector<const TurnRestriction*> restrictions;
    std::vector<int> edges;
};

struct RestrictionGraph {
    std::unordered_map<std::pair<NodeID, NodeID>, int, boost::hash<std::pair<NodeID, NodeID>>> start_edge_to_restriction;
    std::multimap<std::pair<NodeID, NodeID>, int> via_edge_to_restriction;

    std::vector<RestrictionWay> start_ways;
    std::vector<RestrictionWay> via_ways;
    std::vector<RestrictionEdge> edges;
};

RestrictionGraph constructRestrictionGraph(const std::vector<TurnRestriction>& restrictions);

} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_RESTRICTION_GRAPH_HPP_
