#include "engine/routing_algorithms/routing_base.hpp"

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

boost::optional<NodeID> getForwardLoopNode(const std::vector<PhantomNode> &source_phantoms, const PhantomNode &target_phantom)
{
    auto node_it = std::find_if(source_phantoms.begin(), source_phantoms.end(), [&] (const PhantomNode& phantom_node) {
        return phantom_node.IsValidForwardSource() && target_phantom.IsValidForwardTarget() &&
               phantom_node.forward_segment_id.id == target_phantom.forward_segment_id.id &&
               phantom_node.GetForwardWeightPlusOffset() > target_phantom.GetForwardWeightPlusOffset();
    });
    if (node_it != source_phantoms.end()) {
        return node_it->forward_segment_id.id;
    }
    return boost::none;
}

boost::optional<NodeID> getBackwardLoopNode(const std::vector<PhantomNode> &source_phantoms, const PhantomNode &target_phantom)
{
    auto node_it = std::find_if(source_phantoms.begin(), source_phantoms.end(), [&] (const PhantomNode& phantom_node) {
      return phantom_node.IsValidReverseSource() && target_phantom.IsValidReverseTarget() &&
             phantom_node.reverse_segment_id.id == target_phantom.reverse_segment_id.id &&
             phantom_node.GetReverseWeightPlusOffset() > target_phantom.GetReverseWeightPlusOffset();
    });
    if (node_it != source_phantoms.end()) {
        return node_it->reverse_segment_id.id;
    }
    return boost::none;
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
