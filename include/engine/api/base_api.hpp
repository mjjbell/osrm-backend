#ifndef ENGINE_API_BASE_API_HPP
#define ENGINE_API_BASE_API_HPP

#include "engine/api/base_parameters.hpp"
#include "engine/api/flatbuffers/fbresult_generated.h"
#include "engine/datafacade/datafacade_base.hpp"

#include "engine/api/json_factory.hpp"
#include "engine/hint.hpp"
#include "util/coordinate_calculation.hpp"

#include <boost/assert.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <memory>
#include <vector>

namespace osrm
{
namespace engine
{
namespace api
{

class BaseAPI
{
  public:
    BaseAPI(const datafacade::BaseDataFacade &facade_, const BaseParameters &parameters_)
        : facade(facade_), parameters(parameters_)
    {
    }

    util::json::Array MakeWaypoints(const std::vector<PhantomEndpointCandidates> &segment_end_coordinates) const
    {
        BOOST_ASSERT(parameters.coordinates.size() > 0);
        BOOST_ASSERT(parameters.coordinates.size() == segment_end_coordinates.size() + 1);

        util::json::Array waypoints;
        waypoints.values.resize(parameters.coordinates.size());
        waypoints.values[0] = MakeWaypoint(segment_end_coordinates.front().source_phantoms);

        auto out_iter = std::next(waypoints.values.begin());
        boost::range::transform(
            segment_end_coordinates, out_iter, [this](const PhantomEndpointCandidates &phantom_pair) {
                return MakeWaypoint(phantom_pair.target_phantoms);
            });
        return waypoints;
    }

    // FIXME: gcc 4.9 does not like MakeWaypoints to be protected
    // protected:
    util::json::Object MakeWaypoint(const PhantomNodeCandidates &phantoms) const
    {
        if (parameters.generate_hints)
        {
            std::vector<Hint> hints;
            std::string name;
            for (const auto &phantom : phantoms) {
                hints.push_back(Hint{phantom, facade.GetCheckSum()});
                name += "/" + facade.GetNameForID(facade.GetNameIndex(phantom.forward_segment_id.id)).to_string();
            }
            // TODO: check forward/reverse
            return json::makeWaypoint(
                phantoms.front().location,
                util::coordinate_calculation::fccApproximateDistance(phantoms.front().location,
                                                                     phantoms.front().input_location),
                name,
                hints);
        }
        else
        {
            std::string name;
            for (const auto &phantom : phantoms) {
                name += facade.GetNameForID(facade.GetNameIndex(phantom.forward_segment_id.id)).to_string();
            }
            // TODO: check forward/reverse
            return json::makeWaypoint(
                phantoms.front().location,
                util::coordinate_calculation::fccApproximateDistance(phantoms.front().location,
                                                                     phantoms.front().input_location),
                name);
        }
    }

    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fbresult::Waypoint>>>
    MakeWaypoints(flatbuffers::FlatBufferBuilder *builder,
                  const std::vector<PhantomEndpointCandidates> &segment_end_coordinates) const
    {
        BOOST_ASSERT(parameters.coordinates.size() > 0);
        BOOST_ASSERT(parameters.coordinates.size() == segment_end_coordinates.size() + 1);

        std::vector<flatbuffers::Offset<fbresult::Waypoint>> waypoints;
        waypoints.resize(parameters.coordinates.size());
        waypoints[0] =
            MakeWaypoint(builder, segment_end_coordinates.front().source_phantoms)->Finish();

        std::transform(segment_end_coordinates.begin(),
                       segment_end_coordinates.end(),
                       std::next(waypoints.begin()),
                       [this, builder](const PhantomEndpointCandidates &phantom_pair) {
                           return MakeWaypoint(builder, phantom_pair.target_phantoms)->Finish();
                       });
        return builder->CreateVector(waypoints);
    }

    // FIXME: gcc 4.9 does not like MakeWaypoints to be protected
    // protected:
    std::unique_ptr<fbresult::WaypointBuilder> MakeWaypoint(flatbuffers::FlatBufferBuilder *builder,
                                                            const PhantomNodeCandidates &phantoms) const
    {

        auto location =
            fbresult::Position(static_cast<double>(util::toFloating(phantoms.front().location.lon)),
                               static_cast<double>(util::toFloating(phantoms.front().location.lat)));

        std::vector<Hint> hints;
        std::string name;
        for (const auto &phantom : phantoms) {
            hints.push_back(Hint{phantom, facade.GetCheckSum()});
            name += "/" + facade.GetNameForID(facade.GetNameIndex(phantom.forward_segment_id.id)).to_string();
        }
        auto name_string = builder->CreateString(name);

        flatbuffers::Offset<flatbuffers::String> hint_string;
        if (parameters.generate_hints)
        {
            hint_string = builder->CreateString(ToBase64(hints));
        }

        auto waypoint = std::make_unique<fbresult::WaypointBuilder>(*builder);
        waypoint->add_location(&location);
        waypoint->add_distance(util::coordinate_calculation::fccApproximateDistance(
            phantoms.front().location, phantoms.front().input_location));
        waypoint->add_name(name_string);
        if (parameters.generate_hints)
        {
            waypoint->add_hint(hint_string);
        }
        return waypoint;
    }

    const datafacade::BaseDataFacade &facade;
    const BaseParameters &parameters;
};

} // namespace api
} // namespace engine
} // namespace osrm

#endif
