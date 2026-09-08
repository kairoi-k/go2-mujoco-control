#pragma once
// Observed-source research geometry, not an observed continuous solid or support certificate.
#include "world_terrain_snapshot.h"
#include <memory>
#include <set>
#include <string>
namespace go2_terrain { namespace stage_c { namespace observed_collision {
struct Query { double world_x=0, world_y=0, radius_m=0; };
struct Reconstruction {
    // Explicitly assumes each selected scalar represents a horizontal cell top,
    // extruded downward by this depth. Its sidewalls and thickness are inferred.
    bool accept_cell_prism_assumption=false;
    double prism_depth_m=0;
    double max_cell_age_s=0;
};
struct CellPrism {
    std::size_t ix=0, iy=0;
    std::array<double,3> center_world{}, half_size{};
    double yaw_rad=0, representative_top_world_m=0;
    double source_min_world_m=0, source_max_world_m=0;
    std::array<double,3> source_normal_world{};
};
class Descriptor;
struct BuildResult;
BuildResult Build(const WorldTerrainSnapshot&, const Query&, const Reconstruction&);
struct CoverageResult {
    bool valid=false;
    double evaluated_at_s=kTerrainMapUnknown;
    WorldTerrainSnapshotError source_error=WorldTerrainSnapshotError::kInvalidInput;
    WorldTerrainSnapshotQueryResult source_query{};
    std::string failure;
};
class Descriptor final {
    // Copies have no mutable source alias, and no model or scene ground truth enters this API.
    const WorldTerrainSnapshot snapshot_;
    const Reconstruction reconstruction_;
    const WorldTerrainSnapshotQueryResult initial_coverage_;
    const std::vector<CellPrism> prisms_;
    const std::set<std::pair<std::size_t,std::size_t>> cells_;
    Descriptor(WorldTerrainSnapshot snapshot,Reconstruction reconstruction,
               WorldTerrainSnapshotQueryResult coverage,std::vector<CellPrism> prisms,
               std::set<std::pair<std::size_t,std::size_t>> cells)
      :snapshot_(std::move(snapshot)),reconstruction_(reconstruction),
       initial_coverage_(std::move(coverage)),prisms_(std::move(prisms)),cells_(std::move(cells)){}
    friend BuildResult Build(const WorldTerrainSnapshot&, const Query&, const Reconstruction&);
    friend CoverageResult QueryCoverage(const Descriptor&,const Query&,double);
public:
    static constexpr bool can_actuate=false;
    static constexpr bool continuous_surface_certified=false;
    static constexpr bool support_certified=false;
    static constexpr bool continuous_sweep_certified=false;
    static constexpr bool vertical_walls_observed=false;
    const std::vector<CellPrism>& prisms()const{return prisms_;}
    const Reconstruction& reconstruction()const{return reconstruction_;}
    const WorldTerrainSnapshotQueryResult& initial_coverage()const{return initial_coverage_;}
    const WorldTerrainSnapshotMetadata& snapshot_metadata()const{return snapshot_.metadata;}
    const char* assumption()const{return "horizontal scalar cell tops; downward prism depth and vertical sidewalls are reconstruction assumptions; source height bounds do not certify a continuous surface";}
};
struct BuildResult {
    std::shared_ptr<const Descriptor> descriptor;
    WorldTerrainSnapshotQueryResult coverage{};
    std::string failure;
    bool ok()const{return bool(descriptor);}
};
inline BuildResult Build(const WorldTerrainSnapshot& snapshot,const Query& query,
                         const Reconstruction& reconstruction) {
    BuildResult out;
    if(!reconstruction.accept_cell_prism_assumption ||
       !std::isfinite(reconstruction.prism_depth_m)||reconstruction.prism_depth_m<=0 ||
       !std::isfinite(reconstruction.max_cell_age_s)||reconstruction.max_cell_age_s<0) {
        out.failure="explicit finite cell-prism reconstruction parameters required";return out;
    }
    // SampleWorldTerrainSnapshot performs shape/registration/source/freshness,
    // overlap-conflict and complete-single-capture checks. Never merge holes.
    out.coverage=SampleWorldTerrainSnapshot(snapshot,query.world_x,query.world_y,
                                            query.radius_m,reconstruction.max_cell_age_s);
    if(!out.coverage.ok()) {out.failure=WorldTerrainSnapshotErrorName(out.coverage.error);return out;}
    // A mixed fixture/estimator snapshot must not acquire observed lidar provenance.
    for(const auto& entry:snapshot.captures) if(entry.model.source!=TerrainSource::kLidar) {
        out.failure="lidar-only source required";return out;
    }
    const WorldTerrainSnapshotEntry* selected=nullptr;
    for(const auto& entry:snapshot.captures)
        if(entry.model.map_sequence==out.coverage.selected_source_sequence)selected=&entry;
    if(!selected) {out.failure="selected capture unavailable";return out;}
    const auto evidence=world_terrain_snapshot_detail::InspectModel(snapshot,selected->model,
        query.world_x,query.world_y,query.radius_m,reconstruction.max_cell_age_s);
    if(!evidence.complete || evidence.cells.empty()) {out.failure="selected capture incomplete";return out;}
    std::vector<CellPrism> prisms;
    std::set<std::pair<std::size_t,std::size_t>> cells;
    for(const auto& cell:evidence.cells) {
        CellPrism prism;
        prism.ix=cell.ix;prism.iy=cell.iy;
        prism.center_world=cell.center_world;
        prism.center_world[2]-=.5*reconstruction.prism_depth_m;
        prism.half_size={.5*selected->model.resolution_m,.5*selected->model.resolution_m,
                         .5*reconstruction.prism_depth_m};
        prism.yaw_rad=selected->model.registration_yaw_rad;
        prism.representative_top_world_m=cell.height_world;
        prism.source_min_world_m=cell.min_height_world;
        prism.source_max_world_m=cell.max_height_world;
        prism.source_normal_world=cell.normal_world;
        if(!std::isfinite(prism.center_world[2])||prism.half_size[0]<=0||prism.half_size[2]<=0) {
            out.failure="unrepresentable prism geometry";return out;
        }
        cells.emplace(prism.ix,prism.iy);prisms.push_back(prism);
    }
    out.descriptor=std::shared_ptr<const Descriptor>(new const Descriptor(
        snapshot,reconstruction,out.coverage,std::move(prisms),std::move(cells)));
    return out;
}
// query_time_s is the actual source-freshness evaluation clock, not a trajectory
// prediction time. A frozen descriptor never freezes cell age for later reuse.
inline CoverageResult QueryCoverage(const Descriptor& descriptor,const Query& query,double query_time_s) {
    CoverageResult out;out.evaluated_at_s=query_time_s;
    if(!std::isfinite(query_time_s) || query_time_s<descriptor.snapshot_.metadata.state_time_s) {
        out.failure="invalid or preceding coverage evaluation time";return out;
    }
    auto evaluated_snapshot=descriptor.snapshot_;
    evaluated_snapshot.metadata.state_time_s=query_time_s;
    out.source_query=SampleWorldTerrainSnapshot(evaluated_snapshot,query.world_x,
        query.world_y,query.radius_m,descriptor.reconstruction_.max_cell_age_s);
    out.source_error=out.source_query.error;
    if(!out.source_query.ok()) {out.failure=WorldTerrainSnapshotErrorName(out.source_error);return out;}
    if(out.source_query.selected_source_sequence!=descriptor.initial_coverage_.selected_source_sequence) {
        out.failure="query requires a different capture";return out;
    }
    for(const auto& entry:descriptor.snapshot_.captures) {
        if(entry.model.map_sequence!=out.source_query.selected_source_sequence)continue;
        const auto evidence=world_terrain_snapshot_detail::InspectModel(evaluated_snapshot,entry.model,
            query.world_x,query.world_y,query.radius_m,descriptor.reconstruction_.max_cell_age_s);
        if(!evidence.complete) {out.failure="selected capture incomplete";return out;}
        for(const auto& cell:evidence.cells) if(!descriptor.cells_.count({cell.ix,cell.iy})) {
            out.failure="query outside built geometry";return out;
        }
        out.valid=true;return out;
    }
    out.failure="selected capture unavailable";return out;
}
}}} // namespace
