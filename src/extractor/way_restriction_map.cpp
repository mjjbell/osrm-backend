#include "extractor/way_restriction_map.hpp"
#include "util/for_each_pair.hpp"

#include <functional>
#include <iterator>
#include <tuple>
#include <utility>

namespace osrm
{
namespace extractor
{

WayRestrictionMap::WayRestrictionMap(
    const RestrictionGraph &restriction_graph): restriction_graph(restriction_graph) {}

std::size_t WayRestrictionMap::NumberOfDuplicatedNodes() const
{
    return restriction_graph.via_ways.size();
}

bool WayRestrictionMap::IsViaWay(const NodeID from, const NodeID to) const
{
    auto edge = std::make_pair(from, to);
    return restriction_graph.via_edge_to_restriction.count(edge) > 0;
}

std::vector<DuplicatedNodeID> WayRestrictionMap::DuplicatedNodeIDs(const NodeID from,
                                                                   const NodeID to) const
{
    auto edge = std::make_pair(from,to);
    auto restriction_ways = restriction_graph.via_edge_to_restriction.equal_range(edge);
    std::vector<DuplicatedNodeID> result;
    result.reserve(std::distance(restriction_ways.first, restriction_ways.second));
    for (auto it = restriction_ways.first; it != restriction_ways.second; ++it) {
        result.push_back(DuplicatedNodeID(it->second));
    }
    return result;
}

bool WayRestrictionMap::IsRestricted(DuplicatedNodeID duplicated_node, const NodeID to) const
{
    // loop over all restrictions associated with the node. Mark as restricted based on
    // is_only/restricted targets
    for (const auto &restriction: restriction_graph.via_ways[duplicated_node].restrictions)
    {
        if (restriction->instructions.is_only && restriction->via_restriction.to != to)
            return true;
        if (!restriction->instructions.is_only && to == restriction->via_restriction.to)
            return true;
    }
    return false;
}

std::vector<const TurnRestriction*> WayRestrictionMap::GetRestrictions(DuplicatedNodeID duplicated_node, const NodeID to) const
{
    std::vector<const TurnRestriction*> result;
    // loop over all restrictions associated with the node. Mark as restricted based on
    // is_only/restricted targets
    for (const auto &restriction: restriction_graph.via_ways[duplicated_node].restrictions)
    {
        if (restriction->instructions.is_only && (restriction->via_restriction.to != to))
        {
            result.push_back(restriction);
        } else if (!restriction->instructions.is_only && (to == restriction->via_restriction.to))
        {
            result.push_back(restriction);
        }
    }
    if (result.empty()) {
        throw("Asking for the restriction of an unrestricted turn. Check with `IsRestricted` before "
              "calling GetRestriction");
    }
    return result;
}

std::vector<WayRestrictionMap::ViaWay> WayRestrictionMap::DuplicatedViaWays() const {
    std::vector<ViaWay> result;
    result.resize(NumberOfDuplicatedNodes());

    for (auto entry: restriction_graph.via_edge_to_restriction) {
        result[entry.second] = {entry.first.first, entry.first.second};
    }
    return result;
}

NodeID WayRestrictionMap::RemapIfRestrictionStart(const NodeID edge_based_node,
                                                  const NodeID node_based_from,
                                                  const NodeID node_based_via,
                                                  const NodeID node_based_to,
                                                  const NodeID number_of_edge_based_nodes) const {

    auto way = std::make_pair(node_based_from, node_based_via);
    if (restriction_graph.start_edge_to_restriction.count(way) > 0) {
        auto start_way_id = restriction_graph.start_edge_to_restriction.find(way);
        for (auto edgeID: restriction_graph.start_ways[start_way_id->second].edges) {
            auto edge = restriction_graph.edges[edgeID];
            if (edge.external_node == node_based_to) {
                return number_of_edge_based_nodes - NumberOfDuplicatedNodes() + edge.restriction_way;
            }
        }
    }
    return edge_based_node;
}

NodeID WayRestrictionMap::RemapIfRestrictionVia(const NodeID edge_based_target_node,
                                                  const DuplicatedNodeID edge_based_via_node,
                                                  const NodeID node_based_to,
                                                  const NodeID number_of_edge_based_nodes) const {

    auto way_id = edge_based_via_node  - number_of_edge_based_nodes + NumberOfDuplicatedNodes();
    BOOST_ASSERT(way_id < restriction_graph.via_ways.size());
    for (auto edgeID: restriction_graph.via_ways[way_id].edges) {
        auto edge = restriction_graph.edges[edgeID];
        if (edge.external_node == node_based_to) {
            return number_of_edge_based_nodes - NumberOfDuplicatedNodes() + edge.restriction_way;
        }
    }
    return edge_based_target_node;
}

} // namespace extractor
} // namespace osrm
