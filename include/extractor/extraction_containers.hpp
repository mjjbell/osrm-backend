#ifndef EXTRACTION_CONTAINERS_HPP
#define EXTRACTION_CONTAINERS_HPP

#include "extractor/internal_extractor_edge.hpp"
#include "extractor/nodes_of_way.hpp"
#include "extractor/query_node.hpp"
#include "extractor/restriction.hpp"
#include "extractor/scripting_environment.hpp"

#include "storage/tar_fwd.hpp"

#include <unordered_map>

namespace osrm
{
namespace extractor
{

/**
 * Uses  memory containers to store all the data that
 * is collected by the extractor callbacks.
 *
 * The data is the filtered, aggregated and finally written to disk.
 */
class ExtractionContainers
{
    using ReferencedWays = std::unordered_map<OSMWayID, NodesOfWay>;

    void PrepareNodes();
    ReferencedWays IdentifyRestrictionWays();
    ReferencedWays IdentifyManeuverOverrideWays();
    void PrepareManeuverOverrides(ReferencedWays &maneuver_override_ways);
    void PrepareRestrictions(ReferencedWays &restriction_ways);
    void PrepareEdges(ScriptingEnvironment &scripting_environment);

    void WriteNodes(storage::tar::FileWriter &file_out) const;
    void WriteEdges(storage::tar::FileWriter &file_out) const;
    void WriteMetadata(storage::tar::FileWriter &file_out) const;
    void WriteCharData(const std::string &file_name);

  public:
    using NodeIDVector = std::vector<OSMNodeID>;
    using NodeVector = std::vector<QueryNode>;
    using EdgeVector = std::vector<InternalExtractorEdge>;
    using AnnotationDataVector = std::vector<NodeBasedEdgeAnnotation>;
    using NameCharData = std::vector<unsigned char>;
    using NameOffsets = std::vector<size_t>;
    using WayIDVector = std::vector<OSMWayID>;
    using WayNodeIDOffsets = std::vector<size_t>;

    std::vector<OSMNodeID> barrier_nodes;
    std::vector<OSMNodeID> traffic_signals;
    NodeIDVector used_node_id_list;
    NodeVector all_nodes_list;
    EdgeVector all_edges_list;
    AnnotationDataVector all_edges_annotation_data_list;
    NameCharData name_char_data;
    NameOffsets name_offsets;
    WayIDVector ways_list;
    // Offsets into used nodes for each way in list
    WayNodeIDOffsets way_node_id_offsets;

    unsigned max_internal_node_id;

    // list of restrictions before we transform them into the output types. Input containers
    // reference OSMNodeIDs. We can only transform them to the correct internal IDs after we've read
    // everything. Without a multi-parse approach, we have to remember the output restrictions
    // before converting them to the internal formats
    std::vector<InputConditionalTurnRestriction> restrictions_list;
    // turn restrictions - both conditional and unconditional
    std::vector<TurnRestriction> turn_restrictions;

    std::vector<InputManeuverOverride> external_maneuver_overrides_list;
    std::vector<UnresolvedManeuverOverride> internal_maneuver_overrides;

    ExtractionContainers();

    void PrepareData(ScriptingEnvironment &scripting_environment,
                     const std::string &osrm_path,
                     const std::string &names_data_path);
};
}
}

#endif /* EXTRACTION_CONTAINERS_HPP */
