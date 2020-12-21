#ifndef OSRM_SHORTEST_PATH_IMPL_HPP
#define OSRM_SHORTEST_PATH_IMPL_HPP

#include "engine/routing_algorithms/shortest_path.hpp"

#include <boost/assert.hpp>
#include <boost/optional.hpp>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

namespace
{
const size_t INVALID_LEG_INDEX = std::numeric_limits<size_t>::max();

// allows a uturn at the target_phantom
// searches source forward/reverse -> target forward/reverse
template <typename Algorithm>
void searchWithUTurn(SearchEngineData<Algorithm> &engine_working_data,
                     const DataFacade<Algorithm> &facade,
                     typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                     typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                     const PhantomNodeCandidates &source_candidates,
                     const PhantomNodeCandidates &target_candidates,
                     const EdgeWeight &total_weight,
                     EdgeWeight &new_total_weight,
                     std::vector<NodeID> &leg_packed_path)
{
    forward_heap.Clear();
    reverse_heap.Clear();

    // Take the shortest possible path to any of the target candidates from the last route.
    // This works for representing u-turns where the target of the last route is the source of the
    // current route, as all candidates will represent the same location.
    // This also works when there is a gap between the routes (e.g. from a matching result) where
    // we can't achieve continuity between phantom nodes anyway.

    for (const auto &candidate : source_candidates)
    {
        if (candidate.IsValidForwardSource())
        {
            forward_heap.Insert(candidate.forward_segment_id.id,
                                total_weight - candidate.GetForwardWeightPlusOffset(),
                                candidate.forward_segment_id.id);
        }

        if (candidate.IsValidReverseSource())
        {
            forward_heap.Insert(candidate.reverse_segment_id.id,
                                total_weight - candidate.GetReverseWeightPlusOffset(),
                                candidate.reverse_segment_id.id);
        }
    }
    for (const auto &candidate : target_candidates)
    {
        if (candidate.IsValidForwardTarget())
        {
            reverse_heap.Insert(candidate.forward_segment_id.id,
                                candidate.GetForwardWeightPlusOffset(),
                                candidate.forward_segment_id.id);
        }
        if (candidate.IsValidReverseTarget())
        {
            reverse_heap.Insert(candidate.reverse_segment_id.id,
                                candidate.GetReverseWeightPlusOffset(),
                                candidate.reverse_segment_id.id);
        }
    }

    search(engine_working_data,
           facade,
           forward_heap,
           reverse_heap,
           new_total_weight,
           leg_packed_path,
           getForwardLoopNodes(source_candidates, target_candidates),
           getBackwardLoopNodes(source_candidates, target_candidates),
           {source_candidates, target_candidates});
}

// searches shortest path between:
// source forward/reverse -> target forward
// source forward/reverse -> target reverse
template <typename Algorithm>
void search(SearchEngineData<Algorithm> &engine_working_data,
            const DataFacade<Algorithm> &facade,
            typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
            typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
            const std::vector<bool> &search_from_forward_node,
            const std::vector<bool> &search_from_reverse_node,
            const std::vector<PhantomNode> &source_candidates,
            const PhantomNode &target_phantom,
            const std::vector<EdgeWeight> &total_weight_to_forward,
            const std::vector<EdgeWeight> &total_weight_to_reverse,
            EdgeWeight &new_total_weight_to_forward,
            EdgeWeight &new_total_weight_to_reverse,
            std::vector<NodeID> &leg_packed_path_forward,
            std::vector<NodeID> &leg_packed_path_reverse)
{
    BOOST_ASSERT(search_from_forward_node.size() == source_candidates.size());
    BOOST_ASSERT(search_from_reverse_node.size() == source_candidates.size());
    if (target_phantom.IsValidForwardTarget())
    {
        forward_heap.Clear();
        reverse_heap.Clear();
        reverse_heap.Insert(target_phantom.forward_segment_id.id,
                            target_phantom.GetForwardWeightPlusOffset(),
                            target_phantom.forward_segment_id.id);

        for (size_t i = 0; i < source_candidates.size(); ++i)
        {
            const auto &candidate = source_candidates[i];
            if (search_from_forward_node[i] && candidate.IsValidForwardSource())
            {
                forward_heap.Insert(candidate.forward_segment_id.id,
                                    total_weight_to_forward[i] -
                                        candidate.GetForwardWeightPlusOffset(),
                                    candidate.forward_segment_id.id);
            }

            if (search_from_reverse_node[i] && candidate.IsValidReverseSource())
            {
                forward_heap.Insert(candidate.reverse_segment_id.id,
                                    total_weight_to_reverse[i] -
                                        candidate.GetReverseWeightPlusOffset(),
                                    candidate.reverse_segment_id.id);
            }
        }

        search(engine_working_data,
               facade,
               forward_heap,
               reverse_heap,
               new_total_weight_to_forward,
               leg_packed_path_forward,
               getForwardLoopNodes(source_candidates, {target_phantom}),
               {},
               {source_candidates, {target_phantom}});
    }

    if (target_phantom.IsValidReverseTarget())
    {
        forward_heap.Clear();
        reverse_heap.Clear();
        reverse_heap.Insert(target_phantom.reverse_segment_id.id,
                            target_phantom.GetReverseWeightPlusOffset(),
                            target_phantom.reverse_segment_id.id);

        BOOST_ASSERT(search_from_forward_node.size() == source_candidates.size());
        for (size_t i = 0; i < source_candidates.size(); ++i)
        {
            const auto &candidate = source_candidates[i];
            if (search_from_forward_node[i] && candidate.IsValidForwardSource())
            {
                forward_heap.Insert(candidate.forward_segment_id.id,
                                    total_weight_to_forward[i] -
                                        candidate.GetForwardWeightPlusOffset(),
                                    candidate.forward_segment_id.id);
            }

            if (search_from_reverse_node[i] && candidate.IsValidReverseSource())
            {
                forward_heap.Insert(candidate.reverse_segment_id.id,
                                    total_weight_to_reverse[i] -
                                        candidate.GetReverseWeightPlusOffset(),
                                    candidate.reverse_segment_id.id);
            }
        }

        search(engine_working_data,
               facade,
               forward_heap,
               reverse_heap,
               new_total_weight_to_reverse,
               leg_packed_path_reverse,
               {},
               getBackwardLoopNodes(source_candidates, {target_phantom}),
               {source_candidates, {target_phantom}});
    }
}

template <typename Algorithm>
void unpackLegs(const DataFacade<Algorithm> &facade,
                const std::vector<PhantomEndpoints> &leg_endpoints,
                const std::vector<size_t> &route_path_indices,
                const std::vector<NodeID> &total_packed_path,
                const std::vector<std::size_t> &packed_leg_begin,
                const EdgeWeight shortest_path_weight,
                InternalRouteResult &raw_route_data)
{
    raw_route_data.unpacked_path_segments.resize(route_path_indices.size());

    raw_route_data.shortest_path_weight = shortest_path_weight;

    for (const auto current_leg : util::irange<std::size_t>(0UL, route_path_indices.size()))
    {
        auto leg_begin =
            total_packed_path.begin() + packed_leg_begin[route_path_indices[current_leg]];
        auto leg_end =
            total_packed_path.begin() + packed_leg_begin[route_path_indices[current_leg] + 1];
        const auto &unpack_phantom_node_pair = leg_endpoints[current_leg];
        unpackPath(facade,
                   leg_begin,
                   leg_end,
                   unpack_phantom_node_pair,
                   raw_route_data.unpacked_path_segments[current_leg]);

        raw_route_data.source_traversed_in_reverse.push_back(
            (*leg_begin != leg_endpoints[current_leg].source_phantom.forward_segment_id.id));
        raw_route_data.target_traversed_in_reverse.push_back(
            (*std::prev(leg_end) !=
             leg_endpoints[current_leg].target_phantom.forward_segment_id.id));
    }
}

template <typename Algorithm>
inline void initializeHeap(SearchEngineData<Algorithm> &engine_working_data,
                           const DataFacade<Algorithm> &facade)
{

    const auto nodes_number = facade.GetNumberOfNodes();
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(nodes_number);
}

template <>
inline void initializeHeap<mld::Algorithm>(SearchEngineData<mld::Algorithm> &engine_working_data,
                                           const DataFacade<mld::Algorithm> &facade)
{

    const auto nodes_number = facade.GetNumberOfNodes();
    const auto border_nodes_number = facade.GetMaxBorderNodeID() + 1;
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(nodes_number, border_nodes_number);
}

template <typename Algorithm>
InternalRouteResult
constructRouteResult(const DataFacade<Algorithm> &facade,
                     const std::vector<PhantomNodeCandidates> &waypoint_candidates,
                     const std::vector<std::size_t> &route_path_indices,
                     const std::vector<NodeID> &packed_paths,
                     const std::vector<size_t> &packed_path_begin,
                     const EdgeWeight min_weight)
{

    InternalRouteResult raw_route_data;
    // Find the start/end phantom nodes
    std::vector<PhantomEndpoints> path_endpoints;
    for (size_t i = 0; i < waypoint_candidates.size() - 1; ++i)
    {
        const auto &source_candidates = waypoint_candidates[i];
        const auto &target_candidates = waypoint_candidates[i + 1];
        const auto path_index = route_path_indices[i];
        const auto start_node = packed_paths[packed_path_begin[path_index]];
        const auto end_node = packed_paths[packed_path_begin[path_index + 1] - 1];

        auto source_it =
            std::find_if(source_candidates.begin(),
                         source_candidates.end(),
                         [&start_node](const auto &source_phantom) {
                             return (start_node == source_phantom.forward_segment_id.id ||
                                     start_node == source_phantom.reverse_segment_id.id);
                         });
        BOOST_ASSERT(source_it != source_candidates.end());

        auto target_it =
            std::find_if(target_candidates.begin(),
                         target_candidates.end(),
                         [&end_node](const auto &target_phantom) {
                             return (end_node == target_phantom.forward_segment_id.id ||
                                     end_node == target_phantom.reverse_segment_id.id);
                         });
        BOOST_ASSERT(source_it != target_candidates.end());

        path_endpoints.push_back({*source_it, *target_it});
    };
    raw_route_data.route_endpoints = path_endpoints;

    unpackLegs(facade,
               path_endpoints,
               route_path_indices,
               packed_paths,
               packed_path_begin,
               min_weight,
               raw_route_data);

    return raw_route_data;
}

// Allows for u-turn at start or end of leg.
template <typename Algorithm>
InternalRouteResult
shortestPathWithWaypointUTurns(SearchEngineData<Algorithm> &engine_working_data,
                               const DataFacade<Algorithm> &facade,
                               const std::vector<PhantomNodeCandidates> &waypoint_candidates)
{

    EdgeWeight total_weight = 0;
    std::vector<NodeID> total_packed_path;
    std::vector<std::size_t> packed_leg_begin;

    initializeHeap(engine_working_data, facade);

    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;

    for (size_t i = 0; i < waypoint_candidates.size() - 1; ++i)
    {
        const auto source_candidates = waypoint_candidates[i];
        const auto target_candidates = waypoint_candidates[i + 1];
        std::vector<NodeID> packed_leg;
        EdgeWeight new_total_weight = INVALID_EDGE_WEIGHT;

        // BOOST_ASSERT( that these are non zero and valid sources and targets)
        searchWithUTurn(engine_working_data,
                        facade,
                        forward_heap,
                        reverse_heap,
                        source_candidates,
                        target_candidates,
                        total_weight,
                        new_total_weight,
                        packed_leg);

        if (new_total_weight == INVALID_EDGE_WEIGHT)
            return {};

        packed_leg_begin.push_back(total_packed_path.size());
        total_packed_path.insert(total_packed_path.end(), packed_leg.begin(), packed_leg.end());
        total_weight = new_total_weight;
    };

    // Add sentinel
    packed_leg_begin.push_back(total_packed_path.size());

    BOOST_ASSERT(packed_leg_begin.size() == waypoint_candidates.size());

    std::vector<std::size_t> sequential_indices(packed_leg_begin.size() - 1);
    std::iota(sequential_indices.begin(), sequential_indices.end(), 0);
    return constructRouteResult(facade,
                                waypoint_candidates,
                                sequential_indices,
                                total_packed_path,
                                packed_leg_begin,
                                total_weight);
}

struct leg_connections
{
    // Index of previous forward/reverse leg that connects to current forward/reverse leg
    boost::optional<size_t> forward_to_forward;
    boost::optional<size_t> reverse_to_forward;
    boost::optional<size_t> forward_to_reverse;
    boost::optional<size_t> reverse_to_reverse;
};

leg_connections getLegConnections(const PhantomNodeCandidates &source_candidates,
                                  const std::vector<NodeID> &packed_leg_to_forward,
                                  const std::vector<NodeID> &packed_leg_to_reverse,
                                  const EdgeWeight new_total_weight_to_forward,
                                  const EdgeWeight new_total_weight_to_reverse)
{
    leg_connections connections;
    for (size_t j = 0; j < source_candidates.size(); ++j)
    {
        const auto &candidate = source_candidates[j];

        if ((new_total_weight_to_forward != INVALID_EDGE_WEIGHT) &&
            candidate.IsValidForwardSource() &&
            packed_leg_to_forward.front() == candidate.forward_segment_id.id)
        {
            BOOST_ASSERT(!connections.forward_to_forward && !connections.reverse_to_forward);
            connections.forward_to_forward = j;
        }
        else if ((new_total_weight_to_forward != INVALID_EDGE_WEIGHT) &&
                 candidate.IsValidReverseSource() &&
                 packed_leg_to_forward.front() == candidate.reverse_segment_id.id)
        {
            BOOST_ASSERT(!connections.forward_to_forward && !connections.reverse_to_forward);
            connections.reverse_to_forward = j;
        }

        if ((new_total_weight_to_reverse != INVALID_EDGE_WEIGHT) &&
            candidate.IsValidForwardSource() &&
            packed_leg_to_reverse.front() == candidate.forward_segment_id.id)
        {
            BOOST_ASSERT(!connections.forward_to_reverse && !connections.reverse_to_reverse);
            connections.forward_to_reverse = j;
        }
        else if ((new_total_weight_to_reverse != INVALID_EDGE_WEIGHT) &&
                 candidate.IsValidReverseSource() &&
                 packed_leg_to_reverse.front() == candidate.reverse_segment_id.id)
        {
            BOOST_ASSERT(!connections.forward_to_reverse && !connections.reverse_to_reverse);
            connections.reverse_to_reverse = j;
        }
    }
    return connections;
}

struct leg_state
{
    std::vector<bool> reached_forward_node_target;
    std::vector<bool> reached_reverse_node_target;
    std::vector<EdgeWeight> total_weight_to_forward;
    std::vector<EdgeWeight> total_weight_to_reverse;
    std::vector<std::size_t> total_nodes_to_forward;
    std::vector<std::size_t> total_nodes_to_reverse;

    void reset()
    {
        reached_forward_node_target.clear();
        reached_reverse_node_target.clear();
        total_weight_to_forward.clear();
        total_weight_to_reverse.clear();
        total_nodes_to_forward.clear();
        total_nodes_to_reverse.clear();
    }
};

struct route_state
{
    // Single vector to track all paths with index to path nodes
    // Leg data tracks route size and links back to previous path in route
    std::vector<NodeID> total_packed_paths;
    // TODO explain indexing
    std::vector<std::size_t> packed_leg_begin;
    std::vector<std::size_t> previous_leg_in_route;

    leg_state last;
    leg_state current;

    size_t current_leg;
    // TODO explain indexing
    size_t previous_leg_path_offset;

    route_state(const PhantomNodeCandidates &init_candidates)
        : current_leg(0), previous_leg_path_offset(0)
    {
        last.total_weight_to_forward.resize(init_candidates.size(), 0);
        last.total_weight_to_reverse.resize(init_candidates.size(), 0);
        std::transform(
            init_candidates.begin(),
            init_candidates.end(),
            std::back_inserter(last.reached_forward_node_target),
            [](const PhantomNode &phantom_node) { return phantom_node.IsValidForwardSource(); });
        std::transform(
            init_candidates.begin(),
            init_candidates.end(),
            std::back_inserter(last.reached_reverse_node_target),
            [](const PhantomNode &phantom_node) { return phantom_node.IsValidReverseSource(); });
    }

    bool completeLeg()
    {
        std::swap(current, last);
        // Reset current state
        current.reset();

        current_leg++;
        previous_leg_path_offset =
            previous_leg_in_route.size() - 2 * last.total_weight_to_forward.size();

        auto can_reach_leg_forward = std::any_of(last.total_weight_to_forward.begin(),
                                                 last.total_weight_to_forward.end(),
                                                 [](auto v) { return v != INVALID_EDGE_WEIGHT; });
        auto can_reach_leg_reverse = std::any_of(last.total_weight_to_reverse.begin(),
                                                 last.total_weight_to_reverse.end(),
                                                 [](auto v) { return v != INVALID_EDGE_WEIGHT; });

        return can_reach_leg_forward || can_reach_leg_reverse;
    }

    void completeSearch()
    {
        // insert sentinel
        packed_leg_begin.push_back(total_packed_paths.size());
        // BOOST_ASSERT(packed_leg_begin.size() == leg_endpoints.size() + 1);
    }

    size_t previousForwardPath(size_t previous_leg) const
    {
        return previous_leg_path_offset + 2 * previous_leg;
    }

    size_t previousReversePath(size_t previous_leg) const
    {
        return previous_leg_path_offset + 2 * previous_leg + 1;
    }

    void addSearchResult(const PhantomNodeCandidates &source_candidates,
                         const PhantomNode &target_phantom,
                         const std::vector<NodeID> &packed_leg_to_forward,
                         const std::vector<NodeID> &packed_leg_to_reverse,
                         EdgeWeight new_total_weight_to_forward,
                         EdgeWeight new_total_weight_to_reverse)
    {

        // we need to figure out how the new legs connect to the previous ones
        if (current_leg > 0)
        {
            const auto leg_connections = getLegConnections(source_candidates,
                                                           packed_leg_to_forward,
                                                           packed_leg_to_reverse,
                                                           new_total_weight_to_forward,
                                                           new_total_weight_to_reverse);

            if (leg_connections.forward_to_forward)
            {
                auto new_total = last.total_nodes_to_forward[*leg_connections.forward_to_forward] +
                                 packed_leg_to_forward.size();
                current.total_nodes_to_forward.push_back(new_total);
                previous_leg_in_route.push_back(
                    previousForwardPath(*leg_connections.forward_to_forward));
            }
            else if (leg_connections.reverse_to_forward)
            {
                auto new_total = last.total_nodes_to_reverse[*leg_connections.reverse_to_forward] +
                                 packed_leg_to_forward.size();
                current.total_nodes_to_forward.push_back(new_total);
                previous_leg_in_route.push_back(
                    previousReversePath(*leg_connections.reverse_to_forward));
            }
            else
            {
                BOOST_ASSERT(new_total_weight_to_forward == INVALID_EDGE_WEIGHT);
                current.total_nodes_to_forward.push_back(0);
                previous_leg_in_route.push_back(INVALID_LEG_INDEX);
            }

            if (leg_connections.forward_to_reverse)
            {
                auto new_total = last.total_nodes_to_forward[*leg_connections.forward_to_reverse] +
                                 packed_leg_to_reverse.size();
                current.total_nodes_to_reverse.push_back(new_total);
                previous_leg_in_route.push_back(
                    previousForwardPath(*leg_connections.forward_to_reverse));
            }
            else if (leg_connections.reverse_to_reverse)
            {
                auto new_total = last.total_nodes_to_reverse[*leg_connections.reverse_to_reverse] +
                                 packed_leg_to_reverse.size();
                current.total_nodes_to_reverse.push_back(new_total);
                previous_leg_in_route.push_back(
                    previousReversePath(*leg_connections.reverse_to_reverse));
            }
            else
            {
                current.total_nodes_to_reverse.push_back(0);
                previous_leg_in_route.push_back(INVALID_LEG_INDEX);
            }
        }
        else
        {
            previous_leg_in_route.push_back(INVALID_LEG_INDEX);
            current.total_nodes_to_forward.push_back(packed_leg_to_forward.size());

            previous_leg_in_route.push_back(INVALID_LEG_INDEX);
            current.total_nodes_to_reverse.push_back(packed_leg_to_reverse.size());
        }

        BOOST_ASSERT(new_total_weight_to_forward == INVALID_EDGE_WEIGHT ||
                     target_phantom.IsValidForwardTarget());
        current.total_weight_to_forward.push_back(new_total_weight_to_forward);
        current.reached_forward_node_target.push_back(new_total_weight_to_forward !=
                                                      INVALID_EDGE_WEIGHT);

        BOOST_ASSERT(new_total_weight_to_reverse == INVALID_EDGE_WEIGHT ||
                     target_phantom.IsValidReverseTarget());
        current.total_weight_to_reverse.push_back(new_total_weight_to_reverse);
        current.reached_reverse_node_target.push_back(new_total_weight_to_reverse !=
                                                      INVALID_EDGE_WEIGHT);

        packed_leg_begin.push_back(total_packed_paths.size());
        total_packed_paths.insert(
            total_packed_paths.end(), packed_leg_to_forward.begin(), packed_leg_to_forward.end());
        packed_leg_begin.push_back(total_packed_paths.size());
        total_packed_paths.insert(
            total_packed_paths.end(), packed_leg_to_reverse.begin(), packed_leg_to_reverse.end());
    }

    std::pair<std::vector<size_t>, EdgeWeight> getMinRoute()
    {
        // We make sure the fastest route is always in packed_legs_to_forward
        auto forward_range = util::irange<std::size_t>(0UL, last.total_weight_to_forward.size());
        auto forward_min =
            std::min_element(forward_range.begin(), forward_range.end(), [&](size_t a, size_t b) {
                return (last.total_weight_to_forward[a] < last.total_weight_to_forward[b] ||
                        (last.total_weight_to_forward[a] == last.total_weight_to_forward[b] &&
                         last.total_nodes_to_forward[a] < last.total_nodes_to_forward[b]));
            });
        auto reverse_range = util::irange<std::size_t>(0UL, last.total_weight_to_reverse.size());
        auto reverse_min =
            std::min_element(reverse_range.begin(), reverse_range.end(), [&](size_t a, size_t b) {
                return (last.total_weight_to_reverse[a] < last.total_weight_to_reverse[b] ||
                        (last.total_weight_to_reverse[a] == last.total_weight_to_reverse[b] &&
                         last.total_nodes_to_reverse[a] < last.total_nodes_to_reverse[b]));
            });

        auto min_weight = INVALID_EDGE_WEIGHT;
        std::vector<size_t> path_indices;
        if (last.total_weight_to_forward[*forward_min] <
                last.total_weight_to_reverse[*reverse_min] ||
            (last.total_weight_to_forward[*forward_min] ==
                 last.total_weight_to_reverse[*reverse_min] &&
             last.total_nodes_to_forward[*forward_min] < last.total_nodes_to_reverse[*reverse_min]))
        {
            // Get path indices for forward
            auto current_path_index = previousForwardPath(*forward_min);
            path_indices.push_back(current_path_index);
            while (previous_leg_in_route[current_path_index] != INVALID_LEG_INDEX)
            {
                current_path_index = previous_leg_in_route[current_path_index];
                path_indices.push_back(current_path_index);
            }
            min_weight = last.total_weight_to_forward[*forward_min];
        }
        else
        {
            // Get path indices for reverse
            auto current_path_index = previousReversePath(*reverse_min);
            path_indices.push_back(current_path_index);
            while (previous_leg_in_route[current_path_index] != INVALID_LEG_INDEX)
            {
                current_path_index = previous_leg_in_route[current_path_index];
                path_indices.push_back(current_path_index);
            }
            min_weight = last.total_weight_to_reverse[*reverse_min];
        }

        std::reverse(path_indices.begin(), path_indices.end());
        return std::make_pair(std::move(path_indices), min_weight);
    }
};

// Subsequent leg must continue on
template <typename Algorithm>
InternalRouteResult
shortestPathWithWaypointContinuation(SearchEngineData<Algorithm> &engine_working_data,
                                     const DataFacade<Algorithm> &facade,
                                     const std::vector<PhantomNodeCandidates> &waypoint_candidates)
{

    route_state route(waypoint_candidates.front());

    initializeHeap(engine_working_data, facade);
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;

    // this implements a dynamic program that finds the shortest route through
    // a list of leg endpoints.
    for (size_t i = 0; i < waypoint_candidates.size() - 1; ++i)
    {
        const auto source_candidates = waypoint_candidates[i];
        const auto target_candidates = waypoint_candidates[i + 1];
        // We assume each source candidate for this leg was a target candidate from the previous
        // leg, and in the same order.
        BOOST_ASSERT(source_candidates.size() == route.last.reached_forward_node_target.size());
        BOOST_ASSERT(source_candidates.size() == route.last.reached_reverse_node_target.size());

        // We only do a leg search if there is a valid source
        BOOST_ASSERT(std::none_of(route.last.reached_forward_node_target.begin(),
                                  route.last.reached_forward_node_target.end(),
                                  [](auto v) { return v; }) ||
                     HasForwardSource(source_candidates));
        BOOST_ASSERT(std::none_of(route.last.reached_reverse_node_target.begin(),
                                  route.last.reached_reverse_node_target.end(),
                                  [](auto v) { return v; }) ||
                     HasReverseSource(source_candidates));

        for (const auto &target_phantom : target_candidates)
        {
            EdgeWeight new_total_weight_to_forward = INVALID_EDGE_WEIGHT;
            EdgeWeight new_total_weight_to_reverse = INVALID_EDGE_WEIGHT;

            std::vector<NodeID> packed_leg_to_forward;
            std::vector<NodeID> packed_leg_to_reverse;

            if (target_phantom.IsValidForwardTarget() || target_phantom.IsValidReverseTarget())
            {
                search(engine_working_data,
                       facade,
                       forward_heap,
                       reverse_heap,
                       route.last.reached_forward_node_target,
                       route.last.reached_reverse_node_target,
                       source_candidates,
                       target_phantom,
                       route.last.total_weight_to_forward,
                       route.last.total_weight_to_reverse,
                       new_total_weight_to_forward,
                       new_total_weight_to_reverse,
                       packed_leg_to_forward,
                       packed_leg_to_reverse);
            }

            route.addSearchResult(source_candidates,
                                  target_phantom,
                                  packed_leg_to_forward,
                                  packed_leg_to_reverse,
                                  new_total_weight_to_forward,
                                  new_total_weight_to_reverse);
        }

        auto has_valid_path = route.completeLeg();
        // No path found for both target nodes?
        if (!has_valid_path)
            return {};
    };

    const auto has_forward_route =
        std::any_of(route.last.total_weight_to_forward.begin(),
                    route.last.total_weight_to_forward.end(),
                    [](const auto weight) { return weight != INVALID_EDGE_WEIGHT; });
    const auto has_reverse_route =
        std::any_of(route.last.total_weight_to_reverse.begin(),
                    route.last.total_weight_to_reverse.end(),
                    [](const auto weight) { return weight != INVALID_EDGE_WEIGHT; });
    BOOST_ASSERT(has_forward_route || has_reverse_route);

    route.completeSearch();
    std::vector<size_t> min_path_indices;
    EdgeWeight min_weight;
    std::tie(min_path_indices, min_weight) = route.getMinRoute();
    BOOST_ASSERT(min_path_indices.size() + 1 == waypoint_candidates.size());

    return constructRouteResult(facade,
                                waypoint_candidates,
                                min_path_indices,
                                route.total_packed_paths,
                                route.packed_leg_begin,
                                min_weight);
}
} // namespace

template <typename Algorithm>
InternalRouteResult
shortestPathSearch(SearchEngineData<Algorithm> &engine_working_data,
                   const DataFacade<Algorithm> &facade,
                   const std::vector<PhantomNodeCandidates> &waypoint_candidates,
                   const boost::optional<bool> continue_straight_at_waypoint)
{
    const bool allow_uturn_at_waypoint =
        !(continue_straight_at_waypoint ? *continue_straight_at_waypoint
                                        : facade.GetContinueStraightDefault());

    if (allow_uturn_at_waypoint)
    {
        return shortestPathWithWaypointUTurns(engine_working_data, facade, waypoint_candidates);
    }
    else
    {
        return shortestPathWithWaypointContinuation(
            engine_working_data, facade, waypoint_candidates);
    }
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif /* OSRM_SHORTEST_PATH_IMPL_HPP */
