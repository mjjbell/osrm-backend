#ifndef ISOCHRONE_ROUTING_HPP
#define ISOCHRONE_ROUTING_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

// Dispatcher function for isochrone search:
template <typename Algorithm>
void isochroneSearch(SearchEngineData<Algorithm> &engine_working_data,
                const DataFacade<Algorithm> &facade,
                const PhantomNode &phantom_node);

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif
