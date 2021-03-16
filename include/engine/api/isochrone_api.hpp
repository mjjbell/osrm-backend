#ifndef ENGINE_API_ISOCHRONE_HPP
#define ENGINE_API_ISOCHRONE_HPP

#include "extractor/maneuver_override.hpp"
#include "engine/api/base_api.hpp"
#include "engine/api/base_result.hpp"
#include "engine/api/json_factory.hpp"
#include "engine/api/isochrone_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_steps.hpp"
#include "engine/guidance/collapse_turns.hpp"
#include "engine/guidance/lane_processing.hpp"
#include "engine/guidance/post_processing.hpp"
#include "engine/guidance/verbosity_reduction.hpp"

#include "engine/internal_route_result.hpp"

#include "guidance/turn_instruction.hpp"

#include "util/coordinate.hpp"
#include "util/integer_range.hpp"
#include "util/json_util.hpp"

#include <iterator>
#include <vector>

namespace osrm
{
namespace engine
{
namespace api
{

class IsochroneAPI : public BaseAPI
{
  public:
    IsochroneAPI(const datafacade::BaseDataFacade &facade_, const IsochroneParameters &parameters_)
        : BaseAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    void MakeResponse() const {}
  protected:
    const IsochroneParameters &parameters;
};

} // namespace api
} // namespace engine
} // namespace osrm

#endif
