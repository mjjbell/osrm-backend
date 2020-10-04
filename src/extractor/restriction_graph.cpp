#include "extractor/restriction_graph.hpp"
#include "util/node_based_graph.hpp"
#include <util/for_each_pair.hpp>

namespace osrm
{
namespace extractor
{

RestrictionGraph constructRestrictionGraph(
    const std::vector<TurnRestriction>& turn_restrictions) {
    RestrictionGraph restrictionGraph;

    util::Log() << "Building restriction graph for " <<  turn_restrictions.size() << " restrictions.";

    // Step 1, build the prefix trees
    for (const auto& restriction : turn_restrictions) {
        auto start_edge = std::make_pair(restriction.via_restriction.from, restriction.via_restriction.via[0]);
        if (restrictionGraph.start_edge_to_restriction.count(start_edge) == 0) {
            auto start_way = restrictionGraph.start_ways.size();
            restrictionGraph.start_ways.push_back(RestrictionWay{{}, {}});
            restrictionGraph.start_edge_to_restriction[start_edge] = start_way;
        }
        NodeID currentWayID = restrictionGraph.start_edge_to_restriction[start_edge];
        bool isStart = true;
        util::for_each_pair(restriction.via_restriction.via.begin(), restriction.via_restriction.via.end(), [&](NodeID edge_from, NodeID edge_to) {
          auto nextEdge = std::make_pair(edge_from, edge_to);
          if (isStart) {
              for (auto edge_id: restrictionGraph.start_ways[currentWayID].edges) {
                  if (restrictionGraph.edges[edge_id].external_node == edge_to) {
                      currentWayID = restrictionGraph.edges[edge_id].restriction_way;
                      isStart = false;
                      return;
                  }
              }

          } else {
              for (auto edge_id: restrictionGraph.via_ways[currentWayID].edges) {
                  if (restrictionGraph.edges[edge_id].external_node == edge_to) {
                      currentWayID = restrictionGraph.edges[edge_id].restriction_way;
                      return;
                  }
              }
          }
//          util::Log() << "Creating a restriction way";
          // Create a new via way
          auto newViaWayID = restrictionGraph.via_ways.size();
          restrictionGraph.via_ways.push_back(RestrictionWay{{}, {}});
          // Create new edge
//          util::Log() << "Creating a new edge";
          auto newEdgeID = restrictionGraph.edges.size();
          restrictionGraph.edges.push_back(RestrictionEdge{edge_to, newViaWayID, false});
          // Update ways
//          util::Log() << "Updating ways";
          if (isStart) {
              restrictionGraph.start_ways[currentWayID].edges.push_back(newEdgeID);
          } else {
              restrictionGraph.via_ways[currentWayID].edges.push_back(newEdgeID);
          }
          restrictionGraph.via_edge_to_restriction.insert({nextEdge, newViaWayID});
//          util::Log() << "nextEdge " << nextEdge.first << " , " << nextEdge.second;
//          util::Log() << "newViaWayID " << newViaWayID;
          currentWayID = newViaWayID;
          isStart = false;
        });

//        util::Log() << "Adding restriction";
        // Logic for last step, the actual restriction
        if (isStart) {
            restrictionGraph.start_ways[currentWayID].restrictions.push_back(&restriction);
        } else {
            restrictionGraph.via_ways[currentWayID].restrictions.push_back(&restriction);
        }
    }

/*
    for (const auto& restriction_way : restrictionGraph.via_ways) {
        util::Log() << "Restriction";
        for (const auto& restriction: restriction_way.restrictions) {
            util::Log() << restriction->via_restriction.to;
            util::Log() << restriction->instructions.is_only;
        }
        util::Log() << "-----";
    }
*/

    // Step 2 Add restriction transfers edges
    for (const auto& restriction : turn_restrictions) {
        auto start_edge = std::make_pair(restriction.via_restriction.from, restriction.via_restriction.via[0]);

        NodeID currentWayID = restrictionGraph.start_edge_to_restriction[start_edge];
        bool isStart = true;

        std::vector<RestrictionWay> restrictionSuffixPaths;

        util::for_each_pair(restriction.via_restriction.via.begin(), restriction.via_restriction.via.end(), [&](NodeID edge_from, NodeID edge_to) {
            auto nextEdge = std::make_pair(edge_from, edge_to);
            auto foundNext = false;
            if (isStart) {
                for (auto edge_id: restrictionGraph.start_ways[currentWayID].edges) {
                    if (restrictionGraph.edges[edge_id].external_node == edge_to) {
                        currentWayID = restrictionGraph.edges[edge_id].restriction_way;
                        isStart = false;
                        foundNext = true;
                        break;
                    }
                }
            } else {
                for (auto edge_id: restrictionGraph.via_ways[currentWayID].edges) {
                    if (restrictionGraph.edges[edge_id].external_node == edge_to) {
                        currentWayID = restrictionGraph.edges[edge_id].restriction_way;
                        foundNext = true;
                        break;
                    }
                }
            }
            if (!foundNext) {
                // Really shouldn't happen
            }

            std::vector<RestrictionWay> newSuffixPaths;
            for (const auto& suffixWay: restrictionSuffixPaths) {
                for (auto edge_id: suffixWay.edges) {
                    if (restrictionGraph.edges[edge_id].external_node == edge_to) {
                        auto restriction_way = restrictionGraph.via_ways[restrictionGraph.edges[edge_id].restriction_way];
                        newSuffixPaths.push_back(restriction_way);
                    }
                }
            }
            if (restrictionGraph.start_edge_to_restriction.count(nextEdge) > 0) {
                auto start_way = restrictionGraph.start_ways[restrictionGraph.start_edge_to_restriction[nextEdge]];
                newSuffixPaths.push_back(start_way);
            }
            restrictionSuffixPaths = newSuffixPaths;

           // For each edge on path, if there is an edge in the suffix path not on the current path, add as a transfer.
           for (const auto& suffixWay : restrictionSuffixPaths) {
               for (auto suffix_edge_id: suffixWay.edges) {
                   auto suffix_edge = restrictionGraph.edges[suffix_edge_id];
                   if (suffix_edge.is_transfer) {
                       continue;
                   }
                   auto possibleTransfer = true;
                    for (auto current_edge_id: restrictionGraph.via_ways[currentWayID].edges) {
                        auto current_edge = restrictionGraph.edges[current_edge_id];
                        if (current_edge.external_node == suffix_edge.external_node) {
                            // Restriction path already takes this edge
                            possibleTransfer = false;
                            break;
                        }
                    }
                    if (possibleTransfer) {
                        // Add the transfer edge
                        auto newEdgeID = restrictionGraph.edges.size();
                        restrictionGraph.edges.push_back(RestrictionEdge{suffix_edge.external_node, suffix_edge.restriction_way, true});
                        restrictionGraph.via_ways[currentWayID].edges.push_back(newEdgeID);
                    }
                }
                // Also add any restrictions.
                for (const auto& suffixRestriction: suffixWay.restrictions) {
                    restrictionGraph.via_ways[currentWayID].restrictions.push_back(suffixRestriction);
                }
           }
       });
    }

    // Step 3 Use this in existing restriction abstractions.
    return restrictionGraph;
}

} // namespace extractor
} // namespace osrm
