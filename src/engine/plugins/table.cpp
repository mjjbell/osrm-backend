#include "engine/plugins/table.hpp"

#include "engine/api/table_api.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/search_engine_data.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/json_container.hpp"
#include "util/string_util.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/assert.hpp>

namespace osrm
{
namespace engine
{
namespace plugins
{

TablePlugin::TablePlugin(const int max_locations_distance_table)
    : max_locations_distance_table(max_locations_distance_table)
{
}

Status TablePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                  const api::TableParameters &params,
                                  osrm::engine::api::ResultT &result) const
{
    if (!algorithms.HasManyToManySearch())
    {
        return Error("NotImplemented",
                     "Many to many search is not implemented for the chosen search algorithm.",
                     result);
    }

    BOOST_ASSERT(params.IsValid());

    if (!CheckAllCoordinates(params.coordinates))
    {
        return Error("InvalidOptions", "Coordinates are invalid", result);
    }

    if (!params.bearings.empty() && params.coordinates.size() != params.bearings.size())
    {
        return Error(
            "InvalidOptions", "Number of bearings does not match number of coordinates", result);
    }

    if (!CheckAlgorithms(params, algorithms, result))
        return Status::Error;

    const auto &facade = algorithms.GetFacade();
    auto phantom_nodes = GetPhantomNodes(facade, params);

    if (phantom_nodes.size() != params.coordinates.size())
    {
        return Error(
            "NoSegment", MissingPhantomErrorMessage(phantom_nodes, params.coordinates), result);
    }

    auto snapped_phantoms = SnapPhantomNodes(phantom_nodes);

    // Empty sources or destinations means the user wants all of them included, respectively
    // The ManyToMany routing algorithm we dispatch to below already handles this perfectly.
    const auto num_sources =
        params.sources.empty() ? snapped_phantoms.size() : params.sources.size();
    const auto num_destinations =
        params.destinations.empty() ? snapped_phantoms.size() : params.destinations.size();

    if (max_locations_distance_table > 0 &&
        ((num_sources * num_destinations) >
         static_cast<std::size_t>(max_locations_distance_table * max_locations_distance_table)))
    {
        return Error("TooBig", "Too many table coordinates", result);
    }

    bool request_distance = params.annotations & api::TableParameters::AnnotationsType::Distance;
    bool request_duration = params.annotations & api::TableParameters::AnnotationsType::Duration;

    std::vector<EdgeDuration> durations;
    std::vector<EdgeDistance> distances;
    std::tie(durations, distances) = algorithms.ManyToManySearch(
        snapped_phantoms, params.sources, params.destinations, request_distance);

    if ((request_duration && durations.empty()) ||
        (request_distance && distances.empty()))
    {
        return Error("NoTable", "No table found", result);
    }

    std::vector<api::TableAPI::TableCellRef> estimated_pairs;

    std::cout << " num_sources " << num_sources << std::endl;
    std::cout << " num_destinations " << num_destinations << std::endl;
    // Scan table for null results - if any exist, replace with distance estimates
    if (params.fallback_speed != INVALID_FALLBACK_SPEED || params.scale_factor != 1)
    {
        std::cout << " scan table " << std::endl;
        for (std::size_t row = 0; row < num_sources; row++)
        {
            std::cout << " row " << row << std::endl;
            for (std::size_t column = 0; column < num_destinations; column++)
            {
                std::cout << " column " << column << std::endl;
                const auto &table_index = row * num_destinations + column;
                std::cout << " duration for " << row << " and " << column << " is " << durations[table_index] << std::endl;
                BOOST_ASSERT(table_index < durations.size());
                if (params.fallback_speed != INVALID_FALLBACK_SPEED && params.fallback_speed > 0 &&
                    durations[table_index] == MAXIMAL_EDGE_DURATION)
                {
                    std::cout << " going to fallback" << std::endl;
                    const auto &source =
                        snapped_phantoms[params.sources.empty() ? row : params.sources[row]];
                    const auto &destination =
                        snapped_phantoms[params.destinations.empty() ? column
                                                                     : params.destinations[column]];

                    auto distance_estimate =
                        params.fallback_coordinate_type ==
                                api::TableParameters::FallbackCoordinateType::Input
                            ? util::coordinate_calculation::fccApproximateDistance(
                                  source.front().input_location, destination.front().input_location)
                            : util::coordinate_calculation::fccApproximateDistance(
                                  source.front().location, destination.front().location);

                    durations[table_index] =
                        distance_estimate / (double)params.fallback_speed;
                    std::cout << " durations is " << durations[table_index] << std::endl;
                    if (!distances.empty())
                    {
                        std::cout << " distances is " << distances[table_index] << std::endl;
                        distances[table_index] = distance_estimate;
                    }

                    estimated_pairs.emplace_back(row, column);
                }
                if (params.scale_factor > 0 && params.scale_factor != 1 &&
                    durations[table_index] != MAXIMAL_EDGE_DURATION &&
                    durations[table_index] != 0)
                {
                    EdgeDuration diff =
                        MAXIMAL_EDGE_DURATION / durations[table_index];

                    if (params.scale_factor >= diff)
                    {
                        durations[table_index] = MAXIMAL_EDGE_DURATION - 1;
                    }
                    else
                    {
                        durations[table_index] = std::lround(
                            durations[table_index] * params.scale_factor);
                    }
                }
            }
        }
    }

    for (const auto duration: durations) {
        std::cout << "od " << duration << std::endl;
    }

    api::TableAPI table_api{facade, params};
    table_api.MakeResponse({durations, distances}, snapped_phantoms, estimated_pairs, result);

    return Status::Ok;
}
} // namespace plugins
} // namespace engine
} // namespace osrm
