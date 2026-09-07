#pragma once
#include "centroidal_subproblem.h"
#include "terrain_model.h"
#include "world_terrain_view.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>
namespace go2_terrain
{
namespace stage_c
{
// Actual-model nominal foot offsets are supplied by the caller. This adapter
// does not reconstruct them from a terrain oracle or run a second FK model.
struct TerrainCandidateReference
{
    // Preferred path: one COM world reference at each event touchdown time.
    std::vector<go2::Vec3> com_world_at_touchdown;
    std::vector<bool> com_world_at_touchdown_valid;
    // Explicit fallback for a caller that owns a constant or linear world
    // command reference. It is never inferred from event targets or terrain.
    bool com_world_valid = false;
    go2::Vec3 com_world{};
    bool com_velocity_world_valid = false;
    go2::Vec3 com_velocity_world{};
    std::array<go2::Vec3, go2::kLegCount> nominal_foot_center_offset_world{};
    std::array<bool, go2::kLegCount> nominal_offset_valid{};
    std::array<double, go2::kLegCount> foot_radius_m{};
    std::array<bool, go2::kLegCount> foot_radius_valid{};
};
struct TerrainCandidateConfig
{
    // A bounded cross around the actual-model nominal foot center. Ordering is
    // part of deterministic candidate identity; no candidate is selected here.
    std::array<std::array<double, 2>, 5> xy_offsets_m{{
        {{0.0, 0.0}}, {{0.02, 0.0}}, {{-0.02, 0.0}},
        {{0.0, 0.02}}, {{0.0, -0.02}}}};
    std::string required_frame = "world";
    // Explicit opt-in to querying a registered heading-relative production
    // grid through the world view. No relabelling or resampling is performed.
    bool allow_registered_heading_frame = false;
    // A schedule event may remain in stance beyond the bounded planning
    // horizon. Opt-in preserves that physical event end while all generated
    // surface evidence remains valid only through prediction_valid_until.
    bool allow_contact_continuation_beyond_horizon = false;
    double minimum_edge_margin_m = 0.0;
    double maximum_slope_rad = 0.50;
    double maximum_surface_height_span_m = 0.02;
    double maximum_roughness_m = 0.01;
    double maximum_map_age_s = 0.20;
    double maximum_cell_age_s = 0.20;
    // These are caller-owned assumptions attached to each candidate surface;
    // they are not measured force or actuator evidence.
    double assumed_friction_mu = 0.8;
    double assumed_max_normal_n = 180.0;
};
struct TerrainCandidateSurfaceMatch
{
    // surface_world is the terrain point P. sphere_center_world is P+r*n.
    TimedPoint surface_world{};
    TimedPoint sphere_center_world{};
    ContactSurface contact_surface{};
    TerrainPatch patch{};
    double collision_radius_m = 0.0;
    bool valid() const
    {
        const Eigen::Vector3d normal = contact_surface.basis_world.col(2);
        return TimedPointValidForRole(
                   surface_world, PointRole::kSurfaceContactPoint,
                   Frame::kWorld) &&
            TimedPointValidForRole(
                sphere_center_world, PointRole::kFootCollisionCenter,
                Frame::kWorld) &&
            patch.valid && patch.all_known &&
            std::isfinite(collision_radius_m) && collision_radius_m > 0.0 &&
            contact_surface.frame == Frame::kWorld &&
            contact_surface.coverage == MapCoverageState::kKnown &&
            contact_surface.map_epoch != 0 &&
            contact_surface.basis_world.allFinite() &&
            std::abs(contact_surface.basis_world.determinant() - 1.0) < 1.0e-10 &&
            (contact_surface.basis_world.transpose() *
                 contact_surface.basis_world - Eigen::Matrix3d::Identity()).norm() <
                1.0e-10 &&
            normal.allFinite() && std::abs(normal.norm() - 1.0) < 1.0e-10 &&
            surface_world.source_time == sphere_center_world.source_time &&
            (Eigen::Vector3d(sphere_center_world.value.x,
                             sphere_center_world.value.y,
                             sphere_center_world.value.z) -
             Eigen::Vector3d(surface_world.value.x, surface_world.value.y,
                             surface_world.value.z) -
             collision_radius_m * normal).norm() < 1.0e-10 &&
            contact_surface.valid_until.value >= 0 &&
            contact_surface.valid_until >= surface_world.source_time &&
            std::isfinite(contact_surface.friction_mu) &&
            contact_surface.friction_mu >= 0.0 &&
            std::isfinite(contact_surface.min_normal_n) &&
            contact_surface.min_normal_n >= 0.0 &&
            std::isfinite(contact_surface.max_normal_n) &&
            contact_surface.max_normal_n >= contact_surface.min_normal_n &&
            contact_surface.max_normal_n > 0.0;
    }
};
struct TerrainCandidateQueryDiagnostic
{
    std::size_t event_index = 0;
    go2::Leg leg = go2::Leg::FR;
    std::size_t candidate_index = 0;
    WorldTerrainQueryDiagnostic query{};
};
struct TerrainCandidateSet
{
    // This is the existing combination-evaluation transport. matched_surfaces
    // has exactly one entry per event_set.candidates entry, in the same order.
    EventCandidateSet event_set{};
    std::vector<TerrainCandidateSurfaceMatch> matched_surfaces;
    bool complete = true;
    bool valid() const
    {
        if (!complete || !event_set.complete ||
            event_set.candidates.size() != matched_surfaces.size())
            return false;
        for (std::size_t i = 0; i < event_set.candidates.size(); ++i)
            if (!event_set.candidates[i].valid() ||
                !event_set.candidates[i].geometry_hard_feasible ||
                !matched_surfaces[i].valid())
                return false;
        return true;
    }
};
struct TerrainCandidateGenerationResult
{
    std::vector<TerrainCandidateSet> sets;
    // Query failures retain event/leg/candidate identity and the exact world
    // and local coordinates used by the world terrain view. This is diagnostic
    // evidence only; it does not turn an unknown query into a candidate.
    std::vector<TerrainCandidateQueryDiagnostic> rejected_query_diagnostics;
    JointPlannerFailure failure = JointPlannerFailure::kInvalidInput;
    bool valid = false;
};
namespace terrain_candidate_detail
{
inline bool FiniteVec3(const go2::Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
        std::isfinite(value.z);
}
inline bool ValidConfig(const TerrainCandidateConfig &config)
{
    if (config.required_frame.empty() ||
        !std::isfinite(config.minimum_edge_margin_m) ||
        config.minimum_edge_margin_m < 0.0 ||
        !std::isfinite(config.maximum_slope_rad) ||
        config.maximum_slope_rad < 0.0 ||
        !std::isfinite(config.maximum_surface_height_span_m) ||
        config.maximum_surface_height_span_m < 0.0 ||
        !std::isfinite(config.maximum_roughness_m) ||
        config.maximum_roughness_m < 0.0 ||
        !std::isfinite(config.maximum_map_age_s) ||
        config.maximum_map_age_s < 0.0 ||
        !std::isfinite(config.maximum_cell_age_s) ||
        config.maximum_cell_age_s < 0.0 ||
        !std::isfinite(config.assumed_friction_mu) ||
        config.assumed_friction_mu < 0.0 ||
        !std::isfinite(config.assumed_max_normal_n) ||
        config.assumed_max_normal_n <= 0.0)
        return false;
    for (const auto &offset : config.xy_offsets_m)
        if (!std::isfinite(offset[0]) || !std::isfinite(offset[1]))
            return false;
    return true;
}
inline bool ValidReference(const TerrainCandidateReference &reference)
{
    if (reference.com_velocity_world_valid &&
        !FiniteVec3(reference.com_velocity_world))
        return false;
    const bool constant_reference = reference.com_world_valid &&
        FiniteVec3(reference.com_world) &&
        (!reference.com_velocity_world_valid ||
         FiniteVec3(reference.com_velocity_world));
    const bool has_partial_per_event_reference =
        reference.com_world_at_touchdown.size() !=
            reference.com_world_at_touchdown_valid.size();
    if (has_partial_per_event_reference)
        return false;
    if (!reference.com_world_at_touchdown.empty())
    {
        for (std::size_t i = 0; i < reference.com_world_at_touchdown.size(); ++i)
            if (!reference.com_world_at_touchdown_valid[i] ||
                !FiniteVec3(reference.com_world_at_touchdown[i]))
                return false;
    }
    if (!constant_reference && reference.com_world_at_touchdown.empty())
        return false;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!reference.nominal_offset_valid[leg] ||
            !FiniteVec3(reference.nominal_foot_center_offset_world[leg]) ||
            !reference.foot_radius_valid[leg] ||
            !std::isfinite(reference.foot_radius_m[leg]) ||
            reference.foot_radius_m[leg] <= 0.0)
            return false;
    }
    return true;
}
inline bool ValidAbsoluteEventTiming(const TouchdownEventTable &events)
{
    if (events.events.empty() || events.events.size() > kStageCMaxEvents)
        return false;
    bool saw_uncommitted = false;
    for (std::size_t i = 0; i < events.events.size(); ++i)
    {
        const auto &event = events.events[i];
        if (event.id.schedule_epoch == 0 || event.id.sequence == 0 ||
            static_cast<std::size_t>(event.id.leg) >= go2::kLegCount ||
            event.touchdown_time.value < 0 ||
            event.contact_interval_end <= event.touchdown_time ||
            (event.liftoff_valid &&
             (event.liftoff_time.value < 0 ||
              event.liftoff_time >= event.touchdown_time)))
            return false;
        if (event.target_world.valid &&
            !TimedPointValidForRole(event.target_world,
                                    PointRole::kSurfaceContactPoint,
                                    Frame::kWorld))
            return false;
        if (i > 0 && events.events[i - 1].touchdown_time >
                          event.touchdown_time)
            return false;
        for (std::size_t j = 0; j < i; ++j)
            if (events.events[j].id == event.id)
                return false;
        if (!event.committed)
            saw_uncommitted = true;
        else if (saw_uncommitted)
            return false;
    }
    return true;
}

inline bool ValidTerrainMetadata(
    const TerrainModel &model, TimeNs source_state_time,
    std::uint64_t map_epoch, const TerrainCandidateConfig &config)
{
    const bool frame_matches=model.frame_id==config.required_frame && model.frame_id=="world";
    const bool registered_heading=config.allow_registered_heading_frame &&
        config.required_frame=="world" && model.frame_id=="base_link";
    if (!model.registered || !model.valid() || (!frame_matches && !registered_heading) ||
        !WorldTerrainMetadataValid(model) ||
        !std::isfinite(model.origin_m[0]) || !std::isfinite(model.origin_m[1]) ||
        !std::isfinite(model.resolution_m) || source_state_time.value < 0 ||
        map_epoch == 0 || model.epoch != map_epoch ||
        !std::isfinite(model.state_stamp_s) ||
        !std::isfinite(model.map_stamp_s) || !std::isfinite(model.age_s))
        return false;
    const double source_s = source_state_time.seconds();
    const double age = source_s - model.map_stamp_s;
    if (!std::isfinite(source_s) || !std::isfinite(age) ||
        std::abs(model.state_stamp_s - source_s) > kTerrainMapTimeToleranceS ||
        age < -kTerrainMapTimeToleranceS ||
        age > config.maximum_map_age_s + kTerrainMapTimeToleranceS ||
        model.age_s < -kTerrainMapTimeToleranceS ||
        model.age_s > config.maximum_map_age_s + kTerrainMapTimeToleranceS ||
        std::abs(model.age_s - std::max(0.0, age)) > kTerrainMapTimeToleranceS)
        return false;
    return true;
}
inline bool MakeBasis(const std::array<double, 3> &normal_array,
                      Eigen::Matrix3d &basis)
{
    Eigen::Vector3d normal(
        normal_array[0], normal_array[1], normal_array[2]);
    if (!normal.allFinite() || normal.norm() < 1.0e-9)
        return false;
    normal.normalize();
    if (normal.z() <= 0.0)
        return false;
    const Eigen::Vector3d reference = std::abs(normal.z()) < 0.90
        ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    Eigen::Vector3d tangent_one = reference.cross(normal);
    if (!tangent_one.allFinite() || tangent_one.norm() < 1.0e-9)
        return false;
    tangent_one.normalize();
    const Eigen::Vector3d tangent_two = normal.cross(tangent_one);
    basis.col(0) = tangent_one;
    basis.col(1) = tangent_two;
    basis.col(2) = normal;
    return basis.allFinite() &&
        (basis.transpose() * basis - Eigen::Matrix3d::Identity()).norm() < 1.0e-10 &&
        std::abs(basis.determinant() - 1.0) < 1.0e-10;
}
inline TimeNs MapSourceTime(const TerrainModel &model)
{
    return TimeNs::FromSeconds(model.map_stamp_s);
}
} // namespace terrain_candidate_detail
// Generate terrain-only alternatives for every absolute touchdown event. The
// caller's prediction_valid_until is a stationary-terrain assumption; this
// function does not infer it from the map or from event timestamps.
inline TerrainCandidateGenerationResult GenerateTerrainCandidates(
    const TerrainModel &terrain, const TouchdownEventTable &events,
    TimeNs source_state_time, std::uint64_t map_epoch,
    const TerrainCandidateReference &reference, TimeNs prediction_valid_until,
    const TerrainCandidateConfig &config = {})
{
    using namespace terrain_candidate_detail;
    TerrainCandidateGenerationResult result;
    // Schedule previews intentionally leave target_world unset. Validate
    // absolute event timing and identity without requiring a terrain oracle.
    if (!ValidConfig(config) || !ValidReference(reference) ||
        !ValidAbsoluteEventTiming(events) || source_state_time.value < 0 ||
        prediction_valid_until < source_state_time ||
        !ValidTerrainMetadata(terrain, source_state_time, map_epoch, config))
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }
    const TimeNs map_source_time = MapSourceTime(terrain);
    if (map_source_time.value < 0)
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }
    const bool has_any_per_event_reference =
        !reference.com_world_at_touchdown.empty() ||
        !reference.com_world_at_touchdown_valid.empty();
    const bool per_event_com =
        reference.com_world_at_touchdown.size() == events.events.size() &&
        reference.com_world_at_touchdown_valid.size() == events.events.size();
    if (has_any_per_event_reference && !per_event_com)
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }
    const bool has_com_reference = per_event_com || reference.com_world_valid;
    if (!has_com_reference)
    {
        result.failure = JointPlannerFailure::kObservationUnavailable;
        return result;
    }
    if (per_event_com)
        for (std::size_t i = 0; i < events.events.size(); ++i)
            if (!reference.com_world_at_touchdown_valid[i] ||
                !FiniteVec3(reference.com_world_at_touchdown[i]))
            {
                result.failure = JointPlannerFailure::kObservationUnavailable;
                return result;
            }
    const bool horizon_ok = std::all_of(
        events.events.begin(), events.events.end(),
        [prediction_valid_until, &config](const TouchdownEvent &event) {
            if (event.touchdown_time > prediction_valid_until)
                return false;
            return config.allow_contact_continuation_beyond_horizon ||
                event.contact_interval_end <= prediction_valid_until;
        });
    if (!horizon_ok)
    {
        result.failure = JointPlannerFailure::kCoverageIncomplete;
        return result;
    }
    result.sets.reserve(events.events.size());
    bool saw_coverage_rejection = false;
    bool saw_empty_set = false;
    for (std::size_t event_index = 0; event_index < events.events.size(); ++event_index)
    {
        const TouchdownEvent &event = events.events[event_index];
        TerrainCandidateSet output_set;
        output_set.event_set.event_id = event.id;
        const std::size_t leg = static_cast<std::size_t>(event.id.leg);
        go2::Vec3 com = per_event_com
            ? reference.com_world_at_touchdown[event_index]
            : reference.com_world;
        if (!per_event_com && reference.com_velocity_world_valid)
        {
            const double dt = event.touchdown_time.seconds() -
                source_state_time.seconds();
            com.x += reference.com_velocity_world.x * dt;
            com.y += reference.com_velocity_world.y * dt;
            com.z += reference.com_velocity_world.z * dt;
        }
        if (!FiniteVec3(com))
        {
            result.failure = JointPlannerFailure::kObservationUnavailable;
            return result;
        }
        const go2::Vec3 nominal = {
            com.x + reference.nominal_foot_center_offset_world[leg].x,
            com.y + reference.nominal_foot_center_offset_world[leg].y,
            com.z + reference.nominal_foot_center_offset_world[leg].z};
        const double radius = reference.foot_radius_m[leg];
        for (std::size_t candidate_index = 0;
             candidate_index < config.xy_offsets_m.size(); ++candidate_index)
        {
            const double x = nominal.x + config.xy_offsets_m[candidate_index][0];
            const double y = nominal.y + config.xy_offsets_m[candidate_index][1];
            TerrainPatch patch;
            WorldTerrainQueryDiagnostic query_diagnostic;
            if (!SampleWorldTerrainPatch(
                    terrain, x, y, radius, config.maximum_cell_age_s, patch,
                    &query_diagnostic) ||
                !patch.valid || !patch.all_known)
            {
                if (query_diagnostic.failure !=
                    WorldTerrainQueryFailure::kNone)
                    result.rejected_query_diagnostics.push_back({
                        event_index, event.id.leg, candidate_index,
                        query_diagnostic});
                saw_coverage_rejection = true;
                continue;
            }
            const double height_span = patch.max_height_m - patch.min_height_m;
            const bool finite_patch_observation =
                std::isfinite(height_span) && std::isfinite(patch.slope_rad) &&
                std::isfinite(patch.roughness_m) &&
                std::isfinite(patch.map_edge_margin_m) &&
                std::isfinite(patch.normal[0]) &&
                std::isfinite(patch.normal[1]) &&
                std::isfinite(patch.normal[2]);
            if (!finite_patch_observation)
            {
                saw_coverage_rejection = true;
                continue;
            }
            if (height_span > config.maximum_surface_height_span_m ||
                patch.slope_rad > config.maximum_slope_rad ||
                patch.roughness_m > config.maximum_roughness_m ||
                patch.map_edge_margin_m < radius + config.minimum_edge_margin_m)
                continue;
            Eigen::Matrix3d basis;
            if (!MakeBasis(patch.normal, basis))
                continue;
            const Eigen::Vector3d normal = basis.col(2);
            const Eigen::Vector3d surface(x, y, patch.center_height_m);
            const Eigen::Vector3d sphere_center = surface + radius * normal;
            if (!surface.allFinite() || !sphere_center.allFinite())
            {
                saw_coverage_rejection = true;
                continue;
            }
            const TimedPoint surface_point{
                {surface.x(), surface.y(), surface.z()}, Frame::kWorld,
                map_source_time, true, PointRole::kSurfaceContactPoint};
            const TimedPoint sphere_point{
                {sphere_center.x(), sphere_center.y(), sphere_center.z()},
                Frame::kWorld, map_source_time, true,
                PointRole::kFootCollisionCenter};
            ContactSurface contact_surface;
            contact_surface.basis_world = basis;
            contact_surface.frame = Frame::kWorld;
            contact_surface.coverage = MapCoverageState::kKnown;
            contact_surface.map_epoch = terrain.epoch;
            contact_surface.valid_until = prediction_valid_until;
            contact_surface.friction_mu = config.assumed_friction_mu;
            contact_surface.min_normal_n = 0.0;
            contact_surface.max_normal_n = config.assumed_max_normal_n;
            StageCCandidate candidate;
            candidate.candidate_id = static_cast<std::uint32_t>(candidate_index + 1);
            candidate.target_world = surface_point;
            candidate.foothold_cost = std::hypot(
                config.xy_offsets_m[candidate_index][0],
                config.xy_offsets_m[candidate_index][1]);
            candidate.edge_margin_m = patch.map_edge_margin_m;
            // Existing transport requires a finite placeholder. This is not
            // an IK/reachability result; geometry_hard_feasible covers only
            // the explicit terrain gates in this generator.
            candidate.reachability_margin_m = 0.0;
            candidate.coverage = MapCoverageState::kKnown;
            candidate.geometry_hard_feasible = true;
            if (!candidate.valid())
            {
                saw_coverage_rejection = true;
                continue;
            }
            output_set.event_set.candidates.push_back(candidate);
            output_set.matched_surfaces.push_back({
                surface_point, sphere_point, contact_surface, patch, radius});
        }
        if (output_set.event_set.candidates.empty())
            saw_empty_set = true;
        result.sets.push_back(std::move(output_set));
    }
    if (saw_empty_set)
    {
        result.failure = saw_coverage_rejection
            ? JointPlannerFailure::kCoverageIncomplete
            : JointPlannerFailure::kNoFeasibleCandidateInSet;
        return result;
    }
    result.valid = std::all_of(
        result.sets.begin(), result.sets.end(),
        [](const TerrainCandidateSet &set) { return set.valid(); });
    result.failure = result.valid ? JointPlannerFailure::kNone
                                  : JointPlannerFailure::kNumericalFailure;
    return result;
}
} // namespace stage_c
} // namespace go2_terrain
