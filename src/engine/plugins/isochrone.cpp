#include "engine/plugins/isochrone.hpp"
#include "engine/api/isochrone_api.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/status.hpp"

#include "util/integer_range.hpp"

#include <string>

namespace osrm
{
namespace engine
{
namespace plugins
{

IsochronePlugin::IsochronePlugin() {}

Status IsochronePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                     const api::IsochroneParameters &isochrone_parameters,
                                     osrm::engine::api::ResultT &result) const
{
    BOOST_ASSERT(isochrone_parameters.IsValid());

    if (!algorithms.HasIsochroneSearch())
    {
        return Error("NotImplemented",
                     "Isochrone search is not implemented for the chosen search algorithm. "
                     "Only two coordinates supported.",
                     result);
    }

    if (isochrone_parameters.coordinates.size() > 1)
    {
        return Error("TooBig",
                     "Number of entries " + std::to_string(isochrone_parameters.coordinates.size()) +
                         " is higher than current maximum (" +
                         std::to_string(1) + ")",
                     result);
    }

    if (!CheckAllCoordinates(isochrone_parameters.coordinates))
    {
        return Error("InvalidValue", "Invalid coordinate value.", result);
    }

    if (!CheckAlgorithms(isochrone_parameters, algorithms, result))
        return Status::Error;

    const auto &facade = algorithms.GetFacade();
    auto phantom_node_pairs = GetPhantomNodes(facade, isochrone_parameters);
    if (phantom_node_pairs.size() != isochrone_parameters.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(phantom_node_pairs, isochrone_parameters.coordinates),
                     result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == isochrone_parameters.coordinates.size());

    auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);

    api::IsochroneAPI isochrone_api{facade, isochrone_parameters};

    algorithms.IsochroneSearch(snapped_phantoms.front());

    isochrone_api.MakeResponse();
    return Status::Ok;
}
} // namespace plugins
} // namespace engine
} // namespace osrm
