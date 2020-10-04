#ifndef OSRM_EXTRACTOR_RESTRICTION_INDEX_HPP_
#define OSRM_EXTRACTOR_RESTRICTION_INDEX_HPP_

#include "extractor/restriction.hpp"
#include "restriction_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/unordered_map.hpp>

#include <utility>
#include <vector>

namespace osrm
{
namespace extractor
{

// allows easy check for whether a node intersection is present at a given intersection
template <typename filter_type>
class RestrictionIndex
{
  public:

    RestrictionIndex(const RestrictionGraph &restrictionGraph) : restrictionGraph(restrictionGraph), filter(filter_type{}){}

    std::vector<const TurnRestriction*> Restrictions(NodeID first, NodeID second) const
    {
/*
        std::cout << "Checking " << first << ", " << second << std::endl;
*/
        std::vector<const TurnRestriction*> res;
        auto edge = std::make_pair(first, second);
        auto start_way = restrictionGraph.start_edge_to_restriction.find(edge);
            if (start_way != restrictionGraph.start_edge_to_restriction.end()) {
                for (const auto& restriction : restrictionGraph.start_ways[start_way->second].restrictions) {
                    if (filter(restriction)) {
 //                     std::cout << "result " << restriction->via_restriction.to << " , " << restriction->instructions.is_only << std::endl;
                        res.push_back(restriction);
                    }
                }
            }
        return res;
    };

  private:
    const RestrictionGraph& restrictionGraph;
    const filter_type filter;
};

struct ConditionalOnly
{
    bool operator()(const TurnRestriction *restriction) const
    {
        return !restriction->instructions.condition.empty();
    };
};

struct UnconditionalOnly
{
    bool operator()(const TurnRestriction *restriction) const
    {
        return restriction->instructions.condition.empty();
    };
};

// check whether a turn is restricted within a restriction_index
template <typename restriction_map_type>
std::pair<bool, const TurnRestriction *>
isRestricted(const NodeID from,
             const NodeID via,
             const NodeID to,
             const restriction_map_type &restriction_map)
{
    const auto range = restriction_map.Restrictions(from, via);

    // check if a given node_restriction is targeting node
    const auto to_is_restricted = [to](const auto &restriction) {
        auto const restricted = restriction->instructions.is_only ? (to != restriction->via_restriction.to) : (to == restriction->via_restriction.to);
        return restricted;
    };

    auto itr = std::find_if(range.begin(), range.end(), to_is_restricted);

    if (itr != range.end())
        return std::make_pair(true, *itr);
    else
        return {false, NULL};
}

using RestrictionMap = RestrictionIndex<UnconditionalOnly>;
using ConditionalRestrictionMap = RestrictionIndex<ConditionalOnly>;

} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_RESTRICTION_INDEX_HPP_
