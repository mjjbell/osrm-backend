#include "engine/routing_algorithms/routing_base.hpp"

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

std::vector<NodeID> getForwardLoopNodes(const PhantomNodeCandidates &source_candidates,
                                        const PhantomNodeCandidates &target_candidates)
{
    std::vector<NodeID> res;
    for (const auto &source_phantom : source_candidates)
    {
        auto requires_loop = std::any_of(
            target_candidates.begin(), target_candidates.end(), [&](const auto &target_phantom) {
                return source_phantom.IsValidForwardSource() &&
                       target_phantom.IsValidForwardTarget() &&
                       source_phantom.forward_segment_id.id ==
                           target_phantom.forward_segment_id.id &&
                       source_phantom.GetForwardWeightPlusOffset() >
                           target_phantom.GetForwardWeightPlusOffset();
            });
        if (requires_loop)
        {
            res.push_back(source_phantom.forward_segment_id.id);
        }
    }
    return res;
}

std::vector<NodeID> getBackwardLoopNodes(const PhantomNodeCandidates &source_candidates,
                                         const PhantomNodeCandidates &target_candidates)
{
    std::vector<NodeID> res;
    for (const auto &source_phantom : source_candidates)
    {
        auto requires_loop = std::any_of(
            target_candidates.begin(), target_candidates.end(), [&](const auto &target_phantom) {
                return source_phantom.IsValidReverseSource() &&
                       target_phantom.IsValidReverseTarget() &&
                       source_phantom.reverse_segment_id.id ==
                           target_phantom.reverse_segment_id.id &&
                       source_phantom.GetReverseWeightPlusOffset() >
                           target_phantom.GetReverseWeightPlusOffset();
            });
        if (requires_loop)
        {
            res.push_back(source_phantom.reverse_segment_id.id);
        }
    }
    return res;
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
