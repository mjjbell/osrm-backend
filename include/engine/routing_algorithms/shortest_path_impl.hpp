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

// const static constexpr bool DO_NOT_FORCE_LOOP = false;

// allows a uturn at the target_phantom
// searches source forward/reverse -> target forward/reverse
template <typename Algorithm>
void searchWithUTurn(SearchEngineData<Algorithm> &engine_working_data,
                     const DataFacade<Algorithm> &facade,
                     typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                     typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                     const std::vector<bool> &search_from_forward_node,
                     const std::vector<bool> &search_from_reverse_node,
                     const bool search_to_forward_node,
                     const bool search_to_reverse_node,
                     const PhantomNodeCandidates &source_candidates,
                     const PhantomNode &target_phantom,
                     const std::vector<EdgeWeight> &total_weight_to_forward,
                     const std::vector<EdgeWeight> &total_weight_to_reverse,
                     EdgeWeight &new_total_weight,
                     std::vector<NodeID> &leg_packed_path)
{
    forward_heap.Clear();
    reverse_heap.Clear();
    for (size_t i = 0; i < source_candidates.size(); ++i) {
        const auto& candidate = source_candidates[i];
        if (search_from_forward_node[i] && candidate.IsValidForwardSource())
        {
            forward_heap.Insert(candidate.forward_segment_id.id,
                                std::min(total_weight_to_forward[i], total_weight_to_reverse[i]) -
                                    candidate.GetForwardWeightPlusOffset(),
                                candidate.forward_segment_id.id);
        }

        if (search_from_reverse_node[i] && candidate.IsValidReverseSource()) {
            forward_heap.Insert(candidate.reverse_segment_id.id,
                                std::min(total_weight_to_forward[i],total_weight_to_reverse[i]) -
                                candidate.GetReverseWeightPlusOffset(),
                                candidate.reverse_segment_id.id);
        }
    }
    if (search_to_forward_node)
    {
        reverse_heap.Insert(target_phantom.forward_segment_id.id,
                            target_phantom.GetForwardWeightPlusOffset(),
                            target_phantom.forward_segment_id.id);
    }
    if (search_to_reverse_node)
    {
        reverse_heap.Insert(target_phantom.reverse_segment_id.id,
                            target_phantom.GetReverseWeightPlusOffset(),
                            target_phantom.reverse_segment_id.id);
    }

    search(engine_working_data,
           facade,
           forward_heap,
           reverse_heap,
           new_total_weight,
           leg_packed_path,
           getForwardLoopNode(source_candidates, target_phantom),
           getBackwardLoopNode(source_candidates, target_phantom),
           {source_candidates, {target_phantom}});
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
            const bool search_to_forward_node,
            const bool search_to_reverse_node,
            const std::vector<PhantomNode> &source_candidates,
            const PhantomNode &target_phantom,
            const std::vector<EdgeWeight> &total_weight_to_forward,
            const std::vector<EdgeWeight> &total_weight_to_reverse,
            int &new_total_weight_to_forward,
            int &new_total_weight_to_reverse,
            std::vector<NodeID> &leg_packed_path_forward,
            std::vector<NodeID> &leg_packed_path_reverse)
{
    if (search_to_forward_node)
    {
        forward_heap.Clear();
        reverse_heap.Clear();
        reverse_heap.Insert(target_phantom.forward_segment_id.id,
                            target_phantom.GetForwardWeightPlusOffset(),
                            target_phantom.forward_segment_id.id);

        for (size_t i = 0; i < source_candidates.size(); ++i) {
            const auto &candidate = source_candidates[i];
            if (search_from_forward_node[i] && candidate.IsValidForwardSource())
            {
                forward_heap.Insert(candidate.forward_segment_id.id,
                                    total_weight_to_forward[i] - candidate.GetForwardWeightPlusOffset(),
                                    candidate.forward_segment_id.id);
            }

            if (search_from_reverse_node[i] && candidate.IsValidReverseSource()) {
                forward_heap.Insert(candidate.reverse_segment_id.id,
                                    total_weight_to_reverse[i] - candidate.GetReverseWeightPlusOffset(),
                                    candidate.reverse_segment_id.id);
            }
        }

        search(engine_working_data,
               facade,
               forward_heap,
               reverse_heap,
               new_total_weight_to_forward,
               leg_packed_path_forward,
               getForwardLoopNode(source_candidates, target_phantom),
               boost::none,
               {source_candidates, {target_phantom}});
    }

    if (search_to_reverse_node)
    {
        forward_heap.Clear();
        reverse_heap.Clear();
        reverse_heap.Insert(target_phantom.reverse_segment_id.id,
                            target_phantom.GetReverseWeightPlusOffset(),
                            target_phantom.reverse_segment_id.id);

        for (size_t i = 0; i < source_candidates.size(); ++i) {
            const auto &candidate = source_candidates[i];
            if (search_from_forward_node[i] && candidate.IsValidForwardSource()) {
                forward_heap.Insert(candidate.forward_segment_id.id,
                                    total_weight_to_forward[i] - candidate.GetForwardWeightPlusOffset(),
                                    candidate.forward_segment_id.id);
            }

            if (search_from_reverse_node[i] && candidate.IsValidReverseSource()) {
                forward_heap.Insert(candidate.reverse_segment_id.id,
                                    total_weight_to_reverse[i] - candidate.GetReverseWeightPlusOffset(),
                                    candidate.reverse_segment_id.id);
            }
        }

        search(engine_working_data,
               facade,
               forward_heap,
               reverse_heap,
               new_total_weight_to_reverse,
               leg_packed_path_reverse,
               boost::none,
               getBackwardLoopNode(source_candidates, target_phantom),
               {source_candidates, {target_phantom}});
    }
}

template <typename Algorithm>
void unpackLegs(const DataFacade<Algorithm> &facade,
                const std::vector<PhantomNodes> &phantom_nodes_vector,
                const std::vector<NodeID> &total_packed_path,
                const std::vector<std::size_t> &packed_leg_begin,
                const EdgeWeight shortest_path_weight,
                InternalRouteResult &raw_route_data)
{
    raw_route_data.unpacked_path_segments.resize(packed_leg_begin.size() - 1);

    raw_route_data.shortest_path_weight = shortest_path_weight;

    for (const auto current_leg : util::irange<std::size_t>(0UL, packed_leg_begin.size() - 1))
    {
        auto leg_begin = total_packed_path.begin() + packed_leg_begin[current_leg];
        auto leg_end = total_packed_path.begin() + packed_leg_begin[current_leg + 1];
        const auto &unpack_phantom_node_pair = phantom_nodes_vector[current_leg];
        unpackPath(facade,
                   leg_begin,
                   leg_end,
                   unpack_phantom_node_pair,
                   raw_route_data.unpacked_path_segments[current_leg]);

        raw_route_data.source_traversed_in_reverse.push_back(
            (*leg_begin != phantom_nodes_vector[current_leg].source_phantom.forward_segment_id.id));
        raw_route_data.target_traversed_in_reverse.push_back(
            (*std::prev(leg_end) !=
             phantom_nodes_vector[current_leg].target_phantom.forward_segment_id.id));
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
} // namespace

template <typename Algorithm>
InternalRouteResult shortestPathSearch(SearchEngineData<Algorithm> &engine_working_data,
                                       const DataFacade<Algorithm> &facade,
                                       const std::vector<PhantomEndpointCandidates> &via_endpoints,
                                       const boost::optional<bool> continue_straight_at_waypoint)
{
    InternalRouteResult raw_route_data;
    const bool allow_uturn_at_waypoint =
        !(continue_straight_at_waypoint ? *continue_straight_at_waypoint
                                        : facade.GetContinueStraightDefault());

    initializeHeap(engine_working_data, facade);

    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;

    std::vector<EdgeWeight> total_weight_to_forward(via_endpoints.front().source_phantoms.size(),0);
    std::vector<EdgeWeight> total_weight_to_reverse(via_endpoints.front().source_phantoms.size(),0);
    std::vector<bool> search_from_forward_node;
    std::transform(via_endpoints.front().source_phantoms.begin(),
                   via_endpoints.front().source_phantoms.end(),
                   std::back_inserter(search_from_forward_node), [](const PhantomNode &phantom_node) {
                       return phantom_node.IsValidForwardSource();
    });
    std::vector<bool> search_from_reverse_node;
    std::transform(via_endpoints.front().source_phantoms.begin(),
                   via_endpoints.front().source_phantoms.end(),
                   std::back_inserter(search_from_reverse_node), [](const PhantomNode &phantom_node) {
                        return phantom_node.IsValidReverseSource();
    });

    std::vector<std::vector<NodeID>> total_packed_path_to_forward;
    std::vector<std::vector<std::size_t>> packed_leg_to_forward_begin;
    std::vector<std::vector<NodeID>> total_packed_path_to_reverse;
    std::vector<std::vector<std::size_t>> packed_leg_to_reverse_begin;

    std::size_t current_leg = 0;
    // this implements a dynamic program that finds the shortest route through
    // a list of vias
    for (const auto &via_endpoint : via_endpoints)
    {
        std::cout << " This via has " << via_endpoint.source_phantoms.size() << " sources" << std::endl;
        std::cout << " This via has " << via_endpoint.target_phantoms.size() << " targets" << std::endl;
        const auto &source_candidates = via_endpoint.source_phantoms;

        // TODO: Make these asserts more useful
        BOOST_ASSERT(std::none_of(search_from_forward_node.begin(), search_from_forward_node.end(), [](auto v){ return v;}) || std::any_of(source_candidates.begin(), source_candidates.end(), [](const PhantomNode &phantom_node) {
                         return phantom_node.IsValidForwardSource();
        }));
        BOOST_ASSERT(std::none_of(search_from_reverse_node.begin(), search_from_reverse_node.end(), [](auto v){ return v;}) || std::any_of(source_candidates.begin(), source_candidates.end(), [](const PhantomNode &phantom_node) {
                         return phantom_node.IsValidReverseSource();
        }));

        std::vector<EdgeWeight> new_total_weight_to_forwards;
        std::vector<EdgeWeight> new_total_weight_to_reverses;
        std::vector<std::vector<NodeID>> packed_leg_to_forwards;
        std::vector<std::vector<NodeID>> packed_leg_to_reverses;

        const auto num_targets = via_endpoint.target_phantoms.size();
        std::vector<std::vector<NodeID>> new_total_packed_path_to_forward(num_targets);
        std::vector<std::vector<std::size_t>> new_packed_leg_to_forward_begin(num_targets);
        std::vector<std::vector<NodeID>> new_total_packed_path_to_reverse(num_targets);
        std::vector<std::vector<std::size_t>> new_packed_leg_to_reverse_begin(num_targets);

        for (const auto &phantom_target: via_endpoint.target_phantoms) {
            EdgeWeight new_total_weight_to_forward = INVALID_EDGE_WEIGHT;
            EdgeWeight new_total_weight_to_reverse = INVALID_EDGE_WEIGHT;

            std::vector<NodeID> packed_leg_to_forward;
            std::vector<NodeID> packed_leg_to_reverse;

            bool search_to_forward_node = phantom_target.IsValidForwardTarget();
            bool search_to_reverse_node = phantom_target.IsValidReverseTarget();

            if (search_to_reverse_node || search_to_forward_node)
            {
                if (allow_uturn_at_waypoint)
                {
                    searchWithUTurn(engine_working_data,
                                    facade,
                                    forward_heap,
                                    reverse_heap,
                                    search_from_forward_node,
                                    search_from_reverse_node,
                                    search_to_forward_node,
                                    search_to_reverse_node,
                                    source_candidates,
                                    phantom_target,
                                    total_weight_to_forward,
                                    total_weight_to_reverse,
                                    new_total_weight_to_forward,
                                    packed_leg_to_forward);
                    // if only the reverse node is valid (e.g. when using the match plugin) we
                    // actually need to move
                    if (!search_to_forward_node)
                    {
                        BOOST_ASSERT(search_to_reverse_node);
                        new_total_weight_to_reverse = new_total_weight_to_forward;
                        packed_leg_to_reverse = packed_leg_to_forward;
                        new_total_weight_to_forward = INVALID_EDGE_WEIGHT;

                        // (*)
                        //
                        //   Below we have to check if new_total_weight_to_forward is invalid.
                        //   This prevents use-after-move on packed_leg_to_forward.
                    }
                    else if (search_to_reverse_node)
                    {
                        new_total_weight_to_reverse = new_total_weight_to_forward;
                        packed_leg_to_reverse = packed_leg_to_forward;
                    }
                }
                else
                {
                    search(engine_working_data,
                           facade,
                           forward_heap,
                           reverse_heap,
                           search_from_forward_node,
                           search_from_reverse_node,
                           search_to_forward_node,
                           search_to_reverse_node,
                           source_candidates,
                           phantom_target,
                           total_weight_to_forward,
                           total_weight_to_reverse,
                           new_total_weight_to_forward,
                           new_total_weight_to_reverse,
                           packed_leg_to_forward,
                           packed_leg_to_reverse);
                }
            }

            new_total_weight_to_forwards.push_back(new_total_weight_to_forward);
            new_total_weight_to_reverses.push_back(new_total_weight_to_reverse);
            packed_leg_to_forwards.push_back(packed_leg_to_forward);
            packed_leg_to_reverses.push_back(packed_leg_to_reverse);
        }


        // Note: To make sure we do not access the moved-from packed_leg_to_forward
        // we guard its access by a check for invalid edge weight. See  (*) above.

        // No path found for both target nodes?
        const auto no_forward_route = std::all_of(new_total_weight_to_forwards.begin(), new_total_weight_to_forwards.end(), [](const auto weight) {
            return weight == INVALID_EDGE_WEIGHT;
        });
        const auto no_reverse_route = std::all_of(new_total_weight_to_reverses.begin(), new_total_weight_to_reverses.end(), [](const auto weight) {
          return weight == INVALID_EDGE_WEIGHT;
        });
        if (no_forward_route && no_reverse_route)
        {
            return raw_route_data;
        }

        // we need to figure out how the new legs connect to the previous ones
        if (current_leg > 0)
        {
            for (size_t i = 0; i < num_targets; ++i)
            {
                boost::optional<size_t> forward_to_forward;
                boost::optional<size_t> reverse_to_forward;
                boost::optional<size_t> forward_to_reverse;
                boost::optional<size_t> reverse_to_reverse;
                for (size_t j = 0; j < source_candidates.size(); ++j) {
                    std::cout << j << " of " << source_candidates.size() << std::endl;
                    const auto &candidate = source_candidates[j];

                    if ((new_total_weight_to_forwards[i] != INVALID_EDGE_WEIGHT) && candidate.IsValidForwardSource() &&
                        packed_leg_to_forwards[i].front() == candidate.forward_segment_id.id) {
                        forward_to_forward = j;
                        std::cout << " found forward to forward " << j << std::endl;
                    }
                    if ((new_total_weight_to_forwards[i] != INVALID_EDGE_WEIGHT) && candidate.IsValidReverseSource() &&
                        packed_leg_to_forwards[i].front() == candidate.reverse_segment_id.id) {
                        reverse_to_forward = j;
                        std::cout << " found reverse to forward " << j << std::endl;
                    }
                    if ((new_total_weight_to_reverses[i] != INVALID_EDGE_WEIGHT) && candidate.IsValidForwardSource() &&
                        packed_leg_to_reverses[i].front() == candidate.forward_segment_id.id) {
                        forward_to_reverse = j;
                        std::cout << " found forward to reverse " << j << std::endl;
                    }
                    if ((new_total_weight_to_reverses[i] != INVALID_EDGE_WEIGHT) && candidate.IsValidReverseSource() &&
                            packed_leg_to_reverses[i].front() == candidate.reverse_segment_id.id) {
                        reverse_to_reverse = j;
                        std::cout << " found reverse to reverse " << j << std::endl;
                    }
                }

                std::cout << " BOOST ASSERT 1" << std::endl;
                BOOST_ASSERT(!(forward_to_forward && reverse_to_forward));
                BOOST_ASSERT(!(forward_to_reverse && reverse_to_reverse));
                std::cout << " BOOST ASSERT 2" << std::endl;

                if (forward_to_forward) {
                    new_total_packed_path_to_forward[i] = total_packed_path_to_forward[*forward_to_forward];
                    new_packed_leg_to_forward_begin[i] = packed_leg_to_forward_begin[*forward_to_forward];
                    std::cout << " adding " << *forward_to_forward <<  " to new forward packed leg " << new_packed_leg_to_forward_begin.size() << std::endl;
                } else if (reverse_to_forward) {
                    new_total_packed_path_to_forward[i] = total_packed_path_to_reverse[*reverse_to_forward];
                    new_packed_leg_to_forward_begin[i] = packed_leg_to_reverse_begin[*reverse_to_forward];
                    std::cout << " adding " << *reverse_to_forward <<  " to new forward packed leg " << new_packed_leg_to_forward_begin.size() << std::endl;
                } else {
                    new_total_packed_path_to_forward[i] = {};
                    new_packed_leg_to_forward_begin[i] = {};
                    std::cout << " adding empty to new forward packed leg " << new_packed_leg_to_forward_begin.size() << std::endl;
                }

                if (forward_to_reverse) {
                    new_total_packed_path_to_reverse[i] = total_packed_path_to_forward[*forward_to_reverse];
                    new_packed_leg_to_reverse_begin[i] = packed_leg_to_forward_begin[*forward_to_reverse];
                    std::cout << " adding " << *forward_to_reverse <<  " to new reverse packed leg " << new_packed_leg_to_reverse_begin.size() << std::endl;
                } else if (reverse_to_reverse) {
                    new_total_packed_path_to_reverse[i] = total_packed_path_to_reverse[*reverse_to_reverse];
                    new_packed_leg_to_reverse_begin[i] = packed_leg_to_reverse_begin[*reverse_to_reverse];
                    std::cout << " adding " << *reverse_to_reverse <<  " to new reverse packed leg " << new_packed_leg_to_reverse_begin.size() << std::endl;
                } else {
                    new_total_packed_path_to_reverse[i] = {};
                    new_packed_leg_to_reverse_begin[i] = {};
                    std::cout << " adding empty to new reverse packed leg " << new_packed_leg_to_reverse_begin.size() << std::endl;
                }
            }
        }

        std::vector<bool> new_search_from_forward_node(num_targets);
        std::vector<bool> new_search_from_reverse_node(num_targets);
        for (size_t i = 0; i < num_targets; ++i) {

            if (new_total_weight_to_forwards[i] != INVALID_EDGE_WEIGHT)
            {
                auto forward_target = via_endpoint.target_phantoms[i];
                BOOST_ASSERT( forward_target.IsValidForwardTarget());

                new_packed_leg_to_forward_begin[i].push_back(new_total_packed_path_to_forward[i].size());
                std::cout << " adding new size " <<  new_total_packed_path_to_forward[i].size() << " to new forward packed leg " << new_packed_leg_to_forward_begin[i].size() << std::endl;
                new_total_packed_path_to_forward[i].insert(new_total_packed_path_to_forward[i].end(), packed_leg_to_forwards[i].begin(), packed_leg_to_forwards[i].end());
                new_search_from_forward_node[i] = true;
            }
            else
            {
                new_total_packed_path_to_forward[i].clear();
                new_packed_leg_to_forward_begin[i].clear();
                new_search_from_forward_node[i] = false;
                std::cout << "clearing new forward packed leg " << new_packed_leg_to_forward_begin[i].size() << std::endl;
            }

            if (new_total_weight_to_reverses[i] != INVALID_EDGE_WEIGHT)
            {
                auto reverse_target = via_endpoint.target_phantoms[i];
                BOOST_ASSERT(reverse_target.IsValidReverseTarget());

                new_packed_leg_to_reverse_begin[i].push_back(new_total_packed_path_to_reverse[i].size());
                std::cout << " adding new size " <<  new_total_packed_path_to_reverse[i].size() << " to new reverse packed leg " << new_packed_leg_to_reverse_begin[i].size() << std::endl;
                new_total_packed_path_to_reverse[i].insert(new_total_packed_path_to_reverse[i].end(),
                                                    packed_leg_to_reverses[i].begin(),
                                                    packed_leg_to_reverses[i].end());
                new_search_from_reverse_node[i] = true;
            }
            else
            {
                new_total_packed_path_to_reverse[i].clear();
                new_packed_leg_to_reverse_begin[i].clear();
                new_search_from_reverse_node[i] = false;
                std::cout << "clearing new reverse packed leg " << new_packed_leg_to_reverse_begin[i].size() << std::endl;
            }

        }
        total_weight_to_forward = new_total_weight_to_forwards;
        total_weight_to_reverse = new_total_weight_to_reverses;

        total_packed_path_to_forward = new_total_packed_path_to_forward;
        packed_leg_to_forward_begin = new_packed_leg_to_forward_begin;
        total_packed_path_to_reverse = new_total_packed_path_to_reverse;
        packed_leg_to_reverse_begin = new_packed_leg_to_reverse_begin;

        for (const auto &forward_begin : packed_leg_to_forward_begin) {
            std::cout << "forward_begin has size " << forward_begin.size() << std::endl;
        }

        for (const auto &reverse_begin : packed_leg_to_reverse_begin) {
            std::cout << "reverse_begin has size " << reverse_begin.size() << std::endl;
        }

        search_from_reverse_node = new_search_from_reverse_node;
        search_from_forward_node = new_search_from_forward_node;

        ++current_leg;
    }

    const auto a_forward_route = std::any_of(total_weight_to_forward.begin(), total_weight_to_forward.end(), [](const auto weight) {
      return weight != INVALID_EDGE_WEIGHT;
    });
    const auto a_reverse_route = std::any_of(total_weight_to_reverse.begin(), total_weight_to_reverse.end(), [](const auto weight) {
      return weight != INVALID_EDGE_WEIGHT;
    });
    BOOST_ASSERT(a_forward_route || a_reverse_route);

    // We make sure the fastest route is always in packed_legs_to_forward
    auto forward_range = util::irange<std::size_t>(0UL, total_weight_to_forward.size());
    auto forward_min = std::min_element(forward_range.begin(), forward_range.end(), [&](size_t a, size_t b) {
      return (total_weight_to_forward[a] < total_weight_to_forward[b] ||
          (total_weight_to_forward[a] == total_weight_to_forward[b] &&
           total_packed_path_to_forward[a].size() < total_packed_path_to_forward[b].size()));
    });
    auto reverse_range = util::irange<std::size_t>(0UL, total_weight_to_reverse.size());
    auto reverse_min = std::min_element(reverse_range.begin(), reverse_range.end(), [&](size_t a, size_t b) {
      return (total_weight_to_reverse[a] < total_weight_to_reverse[b] ||
              (total_weight_to_reverse[a] == total_weight_to_reverse[b] &&
               total_packed_path_to_reverse[a].size() < total_packed_path_to_reverse[b].size()));
    });

    if (total_weight_to_forward[*forward_min] < total_weight_to_reverse[*reverse_min] ||
        (total_weight_to_forward[*forward_min] == total_weight_to_reverse[*reverse_min] &&
         total_packed_path_to_forward[*forward_min].size() < total_packed_path_to_reverse[*reverse_min].size()))
    {
        // insert sentinel
        packed_leg_to_forward_begin[*forward_min].push_back(total_packed_path_to_forward[*forward_min].size());
        BOOST_ASSERT(packed_leg_to_forward_begin[*forward_min].size() == via_endpoints.size() + 1);

        // Find the start/end phantom nodes
        std::vector<PhantomNodes> path_phantoms;
        for (size_t i = 0; i < via_endpoints.size(); ++i) {
            const auto start_node = total_packed_path_to_forward[*forward_min][packed_leg_to_forward_begin[*forward_min][i]];
            const auto end_node = total_packed_path_to_forward[*forward_min][packed_leg_to_forward_begin[*forward_min][i+1]-1];

            boost::optional<PhantomNode> shortest_source;
            for(const PhantomNode &source_phantom : via_endpoints[i].source_phantoms) {
                if (start_node == source_phantom.forward_segment_id.id || start_node == source_phantom.reverse_segment_id.id) {
                    shortest_source = source_phantom;
                    break;
                }
            };
            BOOST_ASSERT(shortest_source);

            boost::optional<PhantomNode> shortest_target;
            for(const PhantomNode &target_phantom : via_endpoints[i].target_phantoms) {
                if (end_node == target_phantom.forward_segment_id.id || end_node == target_phantom.reverse_segment_id.id) {
                    shortest_target = target_phantom;
                    break;
                }
            };
            BOOST_ASSERT(shortest_target);

            path_phantoms.push_back({*shortest_source, *shortest_target});
        }
        raw_route_data.segment_end_coordinates = path_phantoms;

        unpackLegs(facade,
                   path_phantoms,
                   total_packed_path_to_forward[*forward_min],
                   packed_leg_to_forward_begin[*forward_min],
                   total_weight_to_forward[*forward_min],
                   raw_route_data);
    }
    else
    {
        // insert sentinel
        packed_leg_to_reverse_begin[*reverse_min].push_back(total_packed_path_to_reverse[*reverse_min].size());
        BOOST_ASSERT(packed_leg_to_reverse_begin[*reverse_min].size() == via_endpoints.size() + 1);

        // Find the start/end phantom nodes

        std::vector<PhantomNodes> path_phantoms;
        for (size_t i = 0; i < via_endpoints.size(); ++i) {
            const auto start_node = total_packed_path_to_reverse[*reverse_min][packed_leg_to_reverse_begin[*reverse_min][i]];
            const auto end_node = total_packed_path_to_reverse[*reverse_min][packed_leg_to_reverse_begin[*reverse_min][i+1]-1];

            boost::optional<PhantomNode> shortest_source;
            for(const PhantomNode &source_phantom : via_endpoints[i].source_phantoms) {
                if (start_node == source_phantom.forward_segment_id.id || start_node == source_phantom.reverse_segment_id.id) {
                    shortest_source = source_phantom;
                    break;
                }
            };
            BOOST_ASSERT(shortest_source);

            boost::optional<PhantomNode> shortest_target;
            for(const PhantomNode &target_phantom : via_endpoints[i].target_phantoms) {
                if (end_node == target_phantom.forward_segment_id.id || end_node == target_phantom.reverse_segment_id.id) {
                    shortest_target = target_phantom;
                    break;
                }
            };

            path_phantoms.push_back({*shortest_source, *shortest_target});
        }
        raw_route_data.segment_end_coordinates = path_phantoms;

        unpackLegs(facade,
                   path_phantoms,
                   total_packed_path_to_reverse[*reverse_min],
                   packed_leg_to_reverse_begin[*reverse_min],
                   total_weight_to_reverse[*reverse_min],
                   raw_route_data);
    }

    return raw_route_data;
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif /* OSRM_SHORTEST_PATH_IMPL_HPP */
