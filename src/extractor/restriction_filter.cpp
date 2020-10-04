#include "extractor/restriction_filter.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"
#include <util/for_each_pair.hpp>

#include <algorithm>
#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{

std::vector<TurnRestriction>
removeInvalidRestrictions(std::vector<TurnRestriction> restrictions,
                          const util::NodeBasedDynamicGraph &node_based_graph)
{
    // definition of what we presume to be a valid via-node restriction
    const auto is_valid_node = [&node_based_graph](const auto &node_restriction) {
      // a valid restriction needs to be connected to both its from and to locations

/*
      util::Log() << "node_restriction details";
      util::Log() << "from " << node_restriction.via_restriction.from;
      util::Log() << "via " << node_restriction.via_restriction.via[0];
      util::Log() << "to " << node_restriction.via_restriction.to;
*/
      bool found_from = false, found_to = false;
      for (auto eid : node_based_graph.GetAdjacentEdgeRange(node_restriction.via_restriction.via[0]))
      {
          const auto target = node_based_graph.GetTarget(eid);
//          util::Log() << "target " << target;
          if (target == node_restriction.via_restriction.from)
              found_from = true;
          if (target == node_restriction.via_restriction.to)
              found_to = true;
      }

      if (!found_from || !found_to)
          return false;

      return true;
    };

    // definition of what we presume to be a valid via-way restriction
    const auto is_valid_way = [&node_based_graph](const auto &way_restriction) {

/*
      util::Log() << "way_restriction details";
      util::Log() << "from " << way_restriction.via_restriction.from;
      for (auto via : way_restriction.via_restriction.via) {
          util::Log() << "via " << via;
      }
      util::Log() << "to " << way_restriction.via_restriction.to;
*/

      const auto start_eid = node_based_graph.FindEdge(way_restriction.via_restriction.from, way_restriction.via_restriction.via[0]);
      // the edge needs to exit (we cannot handle intermediate stuff, so far)

      if (start_eid == SPECIAL_EDGEID) {
//          util::Log() << "filter: Invalid start edge" << way_restriction.via_restriction.from << "," << way_restriction.via_restriction.via[0];
          return false;
      }

      const auto &start_data = node_based_graph.GetEdgeData(start_eid);

      // edge needs to be traversable for a valid restriction
      if (start_data.reversed) {
//          util::Log() << "filter: Reversed start edge";
          return false;
      }

      bool edge_invalid = false;
      util::for_each_pair(way_restriction.via_restriction.via.begin(), way_restriction.via_restriction.via.end(), [&](auto via_from, auto via_to) {
        const auto via_eid = node_based_graph.FindEdge(via_from, via_to);
        // the edge needs to exit (we cannot handle intermediate stuff, so far)

        if (via_eid == SPECIAL_EDGEID) {
            edge_invalid = true;
//            util::Log() << "filter: Invalid via edge" << via_from << "," << via_to;
            return;
        }

        const auto &via_data = node_based_graph.GetEdgeData(via_eid);

        // edge needs to be traversable for a valid restriction
        if (via_data.reversed) {
            edge_invalid = true;
//            util::Log() << "filter: Reversed via edge";
            return;
        }
      });

      if (edge_invalid) {
          return false;
      }

      const auto end_eid = node_based_graph.FindEdge(way_restriction.via_restriction.via.back(), way_restriction.via_restriction.to);
      // the edge needs to exit (we cannot handle intermediate stuff, so far)

      if (end_eid == SPECIAL_EDGEID) {
//          util::Log() << "filter: Invalid end edge" << way_restriction.via_restriction.via.back() << "," << way_restriction.via_restriction.to;
          return false;
      }

      const auto &end_data = node_based_graph.GetEdgeData(end_eid);

      // edge needs to be traversable for a valid restriction
      if (end_data.reversed) {
//          util::Log() << "filter: Reversed end edge";
          return false;
      }

      return true;
    };

    const auto is_invalid = [is_valid_way, is_valid_node](const auto &restriction) {
      if (restriction.via_restriction.via.size() == 1)
      {
          return !is_valid_node(restriction);
      }
      else
      {
          BOOST_ASSERT(restriction.via_restriction.via.size() > 1);
          return !is_valid_way(restriction);
      }
    };

    const auto end_valid_restrictions =
        std::remove_if(restrictions.begin(), restrictions.end(), is_invalid);
    restrictions.erase(end_valid_restrictions, restrictions.end());

    return restrictions;
}

} // namespace extractor
} // namespace osrm
