#include "engine/routing_algorithms/isochrone.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

#include <boost/assert.hpp>
#include <boost/range/iterator_range_core.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

namespace mld
{
} // namespace mld

void relaxBorderEdges(const DataFacade<mld::Algorithm> &facade,
                      const NodeID node,
                      const EdgeWeight weight,
                      const EdgeDuration duration,
                      const EdgeDistance distance,
                      typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap &query_heap,
                      LevelID level,
                      int32_t range_duration)
{
    for (const auto edge : facade.GetBorderEdgeRange(level, node))
    {
        const auto &data = facade.GetEdgeData(edge);
        if (facade.IsForwardEdge(edge))
        {
            const NodeID to = facade.GetTarget(edge);
            if (facade.ExcludeNode(to))
            {
                continue;
            }

            const auto turn_id = data.turn_id;
            const auto node_id = node;
            const auto node_weight = facade.GetNodeWeight(node_id);
            const auto node_duration = facade.GetNodeDuration(node_id);
            const auto node_distance = facade.GetNodeDistance(node_id);
            const auto turn_weight = node_weight + facade.GetWeightPenaltyForEdgeID(turn_id);
            const auto turn_duration = node_duration + facade.GetDurationPenaltyForEdgeID(turn_id);

            BOOST_ASSERT_MSG(node_weight + turn_weight > 0, "edge weight is invalid");
            const auto to_weight = weight + turn_weight;
            const auto to_duration = duration + turn_duration;
            const auto to_distance = distance + node_distance;

            if (to_duration > range_duration) {
                // TODO: Add border edge?
                continue;
            }

            // New Node discovered -> Add to Heap + Node Info Storage
            const auto toHeapNode = query_heap.GetHeapNodeIfWasInserted(to);
            if (!toHeapNode)
            {
                query_heap.Insert(to, to_weight, {node, false, to_duration, to_distance});
            }
                // Found a shorter Path -> Update weight and set new parent
            else if (std::tie(to_weight, to_duration, to_distance, node) <
                     std::tie(toHeapNode->weight,
                              toHeapNode->data.duration,
                              toHeapNode->data.distance,
                              toHeapNode->data.parent))
            {
                toHeapNode->data = {node, false, to_duration, to_distance};
                toHeapNode->weight = to_weight;
                query_heap.DecreaseKey(*toHeapNode);
            }
        }
    }
}

template <typename... Args>
void relaxOutgoingEdges(
    const DataFacade<mld::Algorithm> &facade,
    const typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap::HeapNode &heapNode,
    typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap &query_heap,
    const PhantomNode &phantom_node,
    std::vector<std::vector<bool>> in,
    std::vector<std::vector<bool>> out,
    const int32_t range_duration)
{
BOOST_ASSERT(!facade.ExcludeNode(heapNode.node));

    const auto &partition = facade.GetMultiLevelPartition();

    const auto level = mld::getNodeQueryLevel(partition, heapNode.node, phantom_node);

    // Break outgoing edges relaxation if node at the restricted level
    if (level == INVALID_LEVEL_ID)
        return;

    const auto &cells = facade.GetCellStorage();
    const auto &metric = facade.GetCellMetric();

    if (level >= 1 && !heapNode.data.from_clique_arc)
    {
        const auto cell_id = partition.GetCell(level, heapNode.node);
        // Do we have nodes within range
        if (heapNode.data.duration <= range_duration) {
            in[level][cell_id] = true;
        }
        const auto &cell = cells.GetCell(metric, level, cell_id);
        const auto *eccentricity = cell.GetEccentricity(heapNode.node);
        // if eccentricity is within limits, need to check all eccentricities of all
        // source and destinations
        bool all_nodes_in_range = heapNode.data.duration + (eccentricity ? *eccentricity : 0) <= range_duration;

        // Shortcuts in forward direction
        auto destination = cell.GetDestinationNodes().begin();
        auto shortcut_durations = cell.GetOutDuration(heapNode.node);
        auto shortcut_distances = cell.GetOutDistance(heapNode.node);
        for (auto shortcut_weight : cell.GetOutWeight(heapNode.node))
        {
            BOOST_ASSERT(destination != cell.GetDestinationNodes().end());
            BOOST_ASSERT(!shortcut_durations.empty());
            BOOST_ASSERT(!shortcut_distances.empty());
            const NodeID to = *destination;

            if (shortcut_weight != INVALID_EDGE_WEIGHT && heapNode.node != to)
            {
                const auto to_weight = heapNode.weight + shortcut_weight;
                const auto to_duration = heapNode.data.duration + shortcut_durations.front();
                const auto to_distance = heapNode.data.distance + shortcut_distances.front();
                const auto toHeapNode = query_heap.GetHeapNodeIfWasInserted(to);
                if (to_duration > range_duration) {
                    continue;
                }
                if (!toHeapNode)
                {
                    query_heap.Insert(
                        to, to_weight, {heapNode.node, true, to_duration, to_distance});
                }
                else if (std::tie(to_weight, to_duration, to_distance, heapNode.node) <
                         std::tie(toHeapNode->weight,
                                  toHeapNode->data.duration,
                                  toHeapNode->data.distance,
                                  toHeapNode->data.parent))
                {
                    toHeapNode->data = {heapNode.node, true, to_duration, to_distance};
                    toHeapNode->weight = to_weight;
                    query_heap.DecreaseKey(*toHeapNode);
                }
            } else if (all_nodes_in_range) {
                const auto toHeapNode = query_heap.GetHeapNodeIfWasInserted(to);
                if (toHeapNode)
                {
                    const auto *to_eccentricity = cell.GetEccentricity(to);
                    all_nodes_in_range &= toHeapNode->data.duration + (to_eccentricity ? *to_eccentricity : 0) <= range_duration;
                }
            }
            ++destination;
            shortcut_durations.advance_begin(1);
            shortcut_distances.advance_begin(1);
        }

        if (all_nodes_in_range) {
            // So far nodes reachable from the heap nodes are within range
            // All destination boundary nodes not reachable from the heap node are within range
            // This leaves us to check source boundary nodes not reachable from the heap node.
            // In practice we just check all source nodes that they are all within range.
            for (const auto &source : cell.GetDestinationNodes()) {
                // TODO: This should be a find_if type of thing
                const auto sourceHeapNode = query_heap.GetHeapNodeIfWasInserted(source);
                if (sourceHeapNode)
                {
                    const auto *source_eccentricity = cell.GetEccentricity(source);
                    all_nodes_in_range &= sourceHeapNode->data.duration + *source_eccentricity <= range_duration;
                }
            }
        }
        // Do we have nodes out (of range)?
        out[level][cell_id] = !all_nodes_in_range;
        BOOST_ASSERT(shortcut_durations.empty());
        BOOST_ASSERT(shortcut_distances.empty());
    }

    relaxBorderEdges(facade,
                     heapNode.node,
                     heapNode.weight,
                     heapNode.data.duration,
                     heapNode.data.distance,
                     query_heap,
                     level,
                     range_duration);
}

void forwardRoutingStep(const DataFacade<mld::Algorithm> &facade,
                        typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap &query_heap,
                        const PhantomNode &phantom_node,
                        std::vector<std::vector<bool>> in,
                        std::vector<std::vector<bool>> out,
                        const int32_t range_duration)
{
    // Take a copy of the extracted node because otherwise could be modified later if toHeapNode is
    // the same
    const auto heapNode = query_heap.DeleteMinGetHeapNode();
    relaxOutgoingEdges(facade, heapNode, query_heap, phantom_node, in, out, range_duration);
}

void downwardRoutingStep(const DataFacade<mld::Algorithm> &facade,
                        typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap &query_heap,
                        const PhantomNode &phantom_node,
                        std::vector<std::vector<bool>> in,
                        std::vector<std::vector<bool>> out,
                        const int32_t range_duration)
{
    // Take a copy of the extracted node because otherwise could be modified later if toHeapNode is
    // the same
    const auto heapNode = query_heap.DeleteMinGetHeapNode();
    relaxOutgoingEdges(facade, heapNode, query_heap, phantom_node, in, out, range_duration);
}

// Dispatcher function for isochrone search:
template <>
void isochroneSearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                 const DataFacade<mld::Algorithm> &facade,
                 const PhantomNode &phantom_node)
{
    // Clear heap and insert source nodes
    engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
        facade.GetNumberOfNodes(), facade.GetMaxBorderNodeID() + 1);

    int32_t range_duration = 60;

    auto &query_heap = *(engine_working_data.many_to_many_heap);

    insertIsochroneSourceInHeap(query_heap, phantom_node);

    std::vector<std::vector<bool>> in;
    std::vector<std::vector<bool>> out;
    const auto &partition = facade.GetMultiLevelPartition();
    for (const size_t l : util::irange<size_t>(0UL, partition.GetNumberOfLevels())) {
        const auto num_cells = partition.GetNumberOfCells(l);
        in.emplace_back(num_cells, false);
        out.emplace_back(num_cells, true);
    }



    // Explore search space
    while (!query_heap.Empty())
    {
        forwardRoutingStep(facade, query_heap, phantom_node, in, out, range_duration);
    }

    // Downward phase by level
    for (const auto level : boost::adaptors::reverse(util::irange<size_t>(1UL, partition.GetNumberOfLevels())))
    {
        std::cout << level << std::endl;
        downwardRoutingStep(facade, query_heap, phantom_node, in, out, range_duration);
    }
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
