#include "engine/routing_algorithms/isochrone.hpp"

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

namespace ch
{
} // namespace ch

// Dispatcher function for isochrone search:
template <>
void isochroneSearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                 const DataFacade<ch::Algorithm> &facade,
                 const PhantomNode &phantom_node)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(1);
    facade.GetNumberOfNodes();
    std::cout << phantom_node.forward_segment_id.id << std::endl;
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
