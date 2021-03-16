#ifndef ISOCHRONE_HPP
#define ISOCHRONE_HPP

#include "engine/plugins/plugin_base.hpp"
#include "engine/api/isochrone_parameters.hpp"
#include "engine/routing_algorithms.hpp"

#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

class IsochronePlugin final : public BasePlugin
{
  public:
    explicit IsochronePlugin();

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::IsochroneParameters &isochrone_parameters,
                         osrm::engine::api::ResultT &json_result) const;
};
} // namespace plugins
} // namespace engine
} // namespace osrm

#endif // ISOCHRONE_HPP
