#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "terrain_feasibility.h"
#include "terrain_motion_plan.h"
#include "terrain_planner.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

unitree_go::msg::dds_::HeightMap_ FlatMap()
{
    unitree_go::msg::dds_::HeightMap_ map;
    map.stamp(10.0);
    map.frame_id("base_link");
    map.resolution(0.05f);
    map.width(24);
    map.height(20);
    map.origin() = {-0.50f, -0.50f};
    map.data().assign(
        static_cast<std::size_t>(map.width()) * map.height(), -0.25f);
    return map;
}

} // namespace

int main()
{
    // S1 keeps the already-loaded front stance through a delayed rear
    // touchdown, yielding the commanded 0.13 m body preview at 0.30 m/s.
    {
        go2_terrain::TerrainContactSchedule schedule;
        schedule.measured_contact = {true, true, false, false};
        schedule.measured_valid = true;
        schedule.planned_valid = true;
        for (std::size_t k = 0; k < 48; ++k)
            schedule.planned_contact[k] =
                k < 4 ? std::array<bool, go2::kLegCount>{true, true, false, false}
                      : std::array<bool, go2::kLegCount>{true, true, true, false};
        const auto before = schedule.planned_contact;
        std::array<bool, go2::kLegCount> required{false, false, true, false};
        const std::array<bool, go2::kLegCount> committed{true, false, false, false};
        std::array<double, go2::kLegCount> touchdown{0.02, 0.04, 0.08, 0.10};
        const std::array<bool, go2::kLegCount> touchdown_valid{true, true, true, true};
        if (!Check(
                go2_terrain::StretchTerrainFrontStanceSchedule(
                    schedule, required, committed, touchdown, touchdown_valid,
                    0.0, 0.30, 0.02, 48),
                "S1 did not stretch a pending rear transition") ||
            !Check(schedule.planned_contact[4] == before[3],
                   "S1 did not retain the pre-event stance row") ||
            !Check(schedule.planned_contact[26] == before[4] &&
                       schedule.planned_contact[27] == before[4],
                   "S1 inserted the rear event at the wrong knot") ||
            !Check(std::abs(touchdown[2] - 0.52) < 1.0e-9,
                   "S1 did not shift the rear touchdown time"))
            return 1;
        go2_terrain::TerrainContactSchedule crawl_schedule;
        crawl_schedule.measured_contact = {true, true, false, false};
        crawl_schedule.measured_valid = true;
        crawl_schedule.planned_valid = true;
        for (std::size_t k = 0; k < 48; ++k)
            crawl_schedule.planned_contact[k] =
                k < 4 ? std::array<bool, go2::kLegCount>{true, true, false, false}
                      : std::array<bool, go2::kLegCount>{true, true, true, false};
        const auto crawl_before = crawl_schedule.planned_contact;
        std::array<double, go2::kLegCount> crawl_touchdown{0.02, 0.04, 0.08, 0.10};
        if (!Check(
                go2_terrain::StretchTerrainFrontStanceSchedule(
                    crawl_schedule, required, committed, crawl_touchdown,
                    touchdown_valid, 0.0, 0.05, 0.02, 48),
                "S1 rejected a crawl-floor advance longer than one horizon") ||
            !Check(crawl_schedule.planned_contact[47] == crawl_before[3],
                   "S1 did not retain the captured stance through crawl horizon") ||
            !Check(crawl_touchdown[2] > 2.0,
                   "S1 did not defer crawl-floor touchdown to the fixed deadline"))
            return 1;

        go2_terrain::TerrainContactSchedule flat = schedule;
        const auto flat_before = flat.planned_contact;
        const std::array<bool, go2::kLegCount> no_transition{};
        if (!Check(
                !go2_terrain::StretchTerrainFrontStanceSchedule(
                    flat, no_transition, no_transition, touchdown,
                    touchdown_valid, 1.0, 0.30, 0.02, 48),
                "S1 unexpectedly changed a flat-ground schedule") ||
            !Check(flat.planned_contact == flat_before,
                   "flat-ground schedule was not bit-identical"))
            return 1;
    }

    // A failed immutable-time handoff must cancel its uncommitted
    // requirement; committed endpoints remain part of the transaction.
    {
        std::array<bool, go2::kLegCount> required{
            true, true, false, false};
        const std::array<bool, go2::kLegCount> committed{
            false, true, false, false};
        std::array<bool, go2::kLegCount> cancelled{false, false, false, false};
        std::array<bool, go2::kLegCount> source_valid{
            true, true, false, false};
        if (!Check(
                go2_terrain::MarkTerrainTransitionLegCancelled(
                    required, committed, cancelled, source_valid, 0) &&
                    required[0] && cancelled[0] && !source_valid[0] && required[1],
                "unexecutable transition did not remain visible") ||
            !Check(
                !go2_terrain::MarkTerrainTransitionLegCancelled(
                    required, committed, cancelled, source_valid, 1) && required[1],
                "committed transition requirement was released") ||
            !Check(
                !go2_terrain::MarkTerrainTransitionLegCancelled(
                    required, committed, cancelled, source_valid, go2::kLegCount),
                "out-of-range transition leg was accepted"))
            return 1;
    }

    {
        const std::array<bool, go2::kLegCount> required{true, true, false, false};
        const std::array<bool, go2::kLegCount> committed{false, true, false, false};
        const std::array<bool, go2::kLegCount> cancelled{true, false, false, false};
        if (go2_terrain::TerrainTransitionComplete(required, committed, cancelled))
            return 1;
    }

    // A later nominal diagonal must not replace an active transfer hold.
    {
        const std::array<bool, go2::kLegCount> held{true, false, false, true};
        const std::array<bool, go2::kLegCount> scheduled{false, true, true, false};
        if (!Check(
                go2_terrain::TerrainTransferHoldSupport(
                    held, scheduled, true) == held,
                "active transfer hold was replaced by scheduled support") ||
            !Check(
                go2_terrain::TerrainTransferHoldSupport(
                    held, scheduled, false) == scheduled,
                "inactive transfer did not capture scheduled support"))
            return 1;
        const std::array<bool, go2::kLegCount> measured{
            false, false, true, false};
        const auto expanded = go2_terrain::TerrainTransferHoldSupport(
            held, scheduled, measured, true);
        if (!Check(
                expanded == std::array<bool, go2::kLegCount>{true, true, true, true},
                "active transfer hold did not retain scheduled/measured support"))
            return 1;
    }

    // Support remains captured while a target waits at its endpoint or while
    // a non-in-flight transaction requirement is still uncommitted.
    {
        const std::array<bool, go2::kLegCount> required{true, true, false, false};
        const std::array<bool, go2::kLegCount> committed{false, true, false, false};
        const std::array<bool, go2::kLegCount> cancelled{false, false, false, false};
        if (!Check(
                go2_terrain::TerrainTransferSupportMustBeKept(
                    required, committed, cancelled, false, false),
                "uncommitted transition did not preserve waiting support") ||
            !Check(
                !go2_terrain::TerrainTransferSupportMustBeKept(
                    required, committed, cancelled, false, true),
                "in-flight swing was incorrectly held as support") ||
            !Check(
                go2_terrain::TerrainTransferSupportMustBeKept(
                    required, committed, cancelled, true, true),
                "endpoint-held target released support"))
            return 1;
        const std::array<bool, go2::kLegCount> all_committed{
            true, true, false, false};
        if (!Check(
                !go2_terrain::TerrainTransferSupportMustBeKept(
                    required, all_committed, cancelled, false, false),
                "committed transition kept stale support") ||
            !Check(
                go2_terrain::TerrainTransferHoldReleaseReady(
                    required, all_committed, cancelled),
                "fully committed transition did not release hold") ||
            !Check(
                !go2_terrain::TerrainTransferHoldReleaseReady(
                    required, committed, cancelled),
                "partially committed transition released hold"))
            return 1;
    }
    auto map = FlatMap();
    const auto built = go2_terrain::BuildTerrainModel(
        &map, 10.04, 1, go2_terrain::TerrainSource::kLidar);
    if (!Check(built.ok(), "flat lidar map did not build") ||
        !Check(built.model.valid(), "flat terrain model is invalid"))
        return 1;

    // Heightmap z re-referencing: a base rise between the map snapshot and
    // the planner snapshot must shift every finite cell by -dz, preserve
    // unknown cells, and leave the map untouched for a zero/non-finite dz.
    {
        auto shifting = FlatMap();
        shifting.data()[0] = std::numeric_limits<float>::quiet_NaN();
        go2_terrain::RereferenceHeightMapZ(&shifting, 0.04);
        if (!Check(std::abs(shifting.data()[1] - (-0.29f)) < 1.0e-6f,
                   "re-referenced cell did not shift by -dz") ||
            !Check(!std::isfinite(shifting.data()[0]),
                   "re-referencing clobbered an unknown cell"))
            return 1;
        const float before = shifting.data()[1];
        go2_terrain::RereferenceHeightMapZ(&shifting, 0.0);
        go2_terrain::RereferenceHeightMapZ(
            &shifting, std::numeric_limits<double>::quiet_NaN());
        go2_terrain::RereferenceHeightMapZ(nullptr, 0.04);
        if (!Check(shifting.data()[1] == before,
                   "zero or non-finite dz still shifted the map"))
            return 1;
    }

    // Two-contact support capsule semantics, pinned against the epoch11
    // crux geometry: a mid-crossing straddle (front foot on the plateau,
    // rear foot still on flat ground) forces the diagonal support line
    // ~46 mm off the COM path.  The gate must accept that forced geometry
    // while still rejecting a support line outside the stance corridor and
    // a COM projected beyond the segment endpoints.
    {
        go2_terrain::TerrainPlannerConfig margin_config;
        std::array<go2::Vec3, go2::kLegCount> feet{};
        std::array<bool, go2::kLegCount> contact{true, false, true, false};
        feet[0] = {0.225, 0.100, 0.0};
        feet[2] = {-0.125, -0.150, 0.0};
        const go2::Vec3 com{0.006, 0.0, 0.0};
        const double straddle_margin = go2_terrain::SupportMargin2D(
            feet, contact, com, margin_config.min_support_margin_m,
            margin_config.max_two_contact_line_error_m);
        if (!Check(straddle_margin >= margin_config.min_support_margin_m,
                   "mid-crossing straddle geometry was rejected"))
            return 1;

        std::array<go2::Vec3, go2::kLegCount> wide_feet{};
        wide_feet[0] = {0.200, 0.300, 0.0};
        wide_feet[2] = {-0.200, 0.300, 0.0};
        const double corridor_margin = go2_terrain::SupportMargin2D(
            wide_feet, contact, com, margin_config.min_support_margin_m,
            margin_config.max_two_contact_line_error_m);
        if (!Check(corridor_margin < margin_config.min_support_margin_m,
                   "support line outside the stance corridor was accepted"))
            return 1;

        std::array<go2::Vec3, go2::kLegCount> short_feet{};
        short_feet[0] = {0.050, 0.050, 0.0};
        short_feet[2] = {0.100, -0.050, 0.0};
        const double endpoint_margin = go2_terrain::SupportMargin2D(
            short_feet, contact, {0.500, 0.0, 0.0},
            margin_config.min_support_margin_m,
            margin_config.max_two_contact_line_error_m);
        if (!Check(endpoint_margin < margin_config.min_support_margin_m,
                   "COM beyond the support segment endpoints was accepted"))
            return 1;
    }

    // Straddle corridor semantics use planner-owned transition intent rather
    // than measured z differences from blended map cells. Exactly one leg
    // committed to the new surface is a crossing; both legs committed to the
    // new surface use the ordinary drift band.
    {
        go2_terrain::TerrainPlannerConfig corridor_config;
        std::array<go2::Vec3, go2::kLegCount> feet{};
        std::array<bool, go2::kLegCount> contact{true, false, true, false};
        std::array<bool, go2::kLegCount> no_transition{};
        std::array<bool, go2::kLegCount> one_transition{
            true, false, false, false};
        std::array<bool, go2::kLegCount> both_transition{
            true, false, true, false};
        std::array<bool, go2::kLegCount> intent_valid{
            true, true, true, true};

        // The measured-support replacement must preserve the one leg still
        // pending even when the other contact has already been confirmed.
        go2_terrain::TerrainPlannerInput measured_input;
        measured_input.terrain_surface_transition_active = true;
        measured_input.terrain_surface_transition_required =
            {true, false, true, false};
        measured_input.terrain_surface_transition_committed =
            {false, false, true, false};
        std::array<bool, go2::kLegCount> measured_required{};
        std::array<bool, go2::kLegCount> measured_valid{};
        go2_terrain::PopulateMeasuredSupportTransitionIntent(
            measured_input, contact, measured_required, measured_valid);
        if (!Check(measured_required[0] && !measured_required[2] &&
                       measured_valid[0] && measured_valid[2],
                   "measured support erased pending transition intent"))
            return 1;
        std::array<bool, go2::kLegCount> missing_intent{
            false, false, true, false};
        feet[0] = {0.200, -0.050, 0.0};
        feet[2] = {-0.200, 0.050, 0.0};
        const go2::Vec3 com{0.0, 0.058, 0.0};  // line error ~56 mm

        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       feet, contact, corridor_config, no_transition, intent_valid) -
                           corridor_config.max_two_contact_line_error_m) <
                       1.0e-12,
                   "flat two-contact set lost the drift band"))
            return 1;
        const double flat_margin = go2_terrain::SupportMargin2D(
            feet, contact, com, corridor_config.min_support_margin_m,
            go2_terrain::TwoContactLineErrorBound(
                feet, contact, corridor_config, no_transition, intent_valid));
        if (!Check(flat_margin < corridor_config.min_support_margin_m,
                   "flat deep-offset support line was accepted"))
            return 1;

        auto straddle_feet = feet;
        straddle_feet[0].z = 0.050;
        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       straddle_feet, contact, corridor_config, one_transition, intent_valid) -
                           corridor_config.two_contact_straddle_corridor_m) <
                       1.0e-12,
                   "straddling two-contact set did not get the corridor"))
            return 1;
        const double straddle_margin = go2_terrain::SupportMargin2D(
            straddle_feet, contact, com,
            corridor_config.min_support_margin_m,
            go2_terrain::TwoContactLineErrorBound(
                straddle_feet, contact, corridor_config, one_transition, intent_valid));
        if (!Check(straddle_margin >= corridor_config.min_support_margin_m,
                   "committed-crossing straddle geometry was rejected"))
            return 1;

        auto coarse_blended_feet = feet;
        coarse_blended_feet[0].z = 0.029;
        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       coarse_blended_feet, contact, corridor_config,
                       one_transition, intent_valid) -
                           corridor_config.two_contact_straddle_corridor_m) <
                       1.0e-12,
                   "coarse blended transition lost its surface intent"))
            return 1;
        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       feet, contact, corridor_config, both_transition, intent_valid) -
                           corridor_config.max_two_contact_line_error_m) <
                       1.0e-12,
                   "two pending transitions incorrectly used straddle corridor"))
            return 1;

        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       feet, contact, corridor_config, one_transition,
                       missing_intent) -
                           corridor_config.max_two_contact_line_error_m) <
                       1.0e-12,
                   "missing transition intent did not fail closed"))
            return 1;

        const go2::Vec3 far_com{0.0, 0.200, 0.0};  // line error ~194 mm
        const double outside_margin = go2_terrain::SupportMargin2D(
            straddle_feet, contact, far_com,
            corridor_config.min_support_margin_m,
            go2_terrain::TwoContactLineErrorBound(
                straddle_feet, contact, corridor_config, one_transition, intent_valid));
        if (!Check(outside_margin < corridor_config.min_support_margin_m,
                   "support line outside the stance corridor was accepted"))
            return 1;

        // A 15 mm riser blend selects the corridor through intent, while
        // a 6 mm flat quantization difference remains on the drift band.
        auto blended_feet = feet;
        blended_feet[0].z = 0.015;
        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       blended_feet, contact, corridor_config, one_transition, intent_valid) -
                           corridor_config.two_contact_straddle_corridor_m) <
                       1.0e-12,
                   "blended riser-edge straddle did not get the corridor"))
            return 1;
        auto flat_quantized_feet = feet;
        flat_quantized_feet[0].z = 0.006;
        if (!Check(std::abs(go2_terrain::TwoContactLineErrorBound(
                       flat_quantized_feet, contact, corridor_config, no_transition, intent_valid) -
                           corridor_config.max_two_contact_line_error_m) <
                       1.0e-12,
                   "flat quantization was misclassified as a straddle"))
            return 1;
    }

    go2_terrain::TerrainPatch patch;
    if (!Check(built.model.SamplePatch(0.20, 0.0, 0.025, patch),
               "flat patch was not sampled") ||
        !Check(patch.all_known && std::abs(patch.slope_rad) < 1e-9,
               "flat patch lost known/plane information"))
        return 1;

    auto unknown_map = map;
    unknown_map.data()[10 * unknown_map.width() + 14] =
        std::numeric_limits<float>::quiet_NaN();
    const auto unknown_built = go2_terrain::BuildTerrainModel(
        &unknown_map, 10.04, 2, go2_terrain::TerrainSource::kLidar);
    go2_terrain::TerrainPatch unknown_patch;
    if (!Check(unknown_built.ok(), "unknown map did not build") ||
        !Check(unknown_built.model.SamplePatch(0.22, 0.0, 0.025,
                                               unknown_patch) &&
                   !unknown_patch.all_known,
               "unknown cell was silently imputed"))
        return 1;

    go2_terrain::TerrainFeasibilityConfig feasibility;
    const auto safe = go2_terrain::EvaluateFoothold(
        built.model, go2::Leg::FR, 0.20, -0.10, feasibility);
    if (!Check(safe.hard_feasible, "flat reachable foothold rejected"))
        return 1;

    auto step_map = map;
    for (std::size_t iy = 0; iy < step_map.height(); ++iy)
    {
        for (std::size_t ix = 16; ix < step_map.width(); ++ix)
            step_map.data()[iy * step_map.width() + ix] = -0.15f;
    }
    const auto step_built = go2_terrain::BuildTerrainModel(
        &step_map, 10.04, 3, go2_terrain::TerrainSource::kLidar);
    const auto edge = go2_terrain::EvaluateFoothold(
        step_built.model, go2::Leg::FR, 0.30, -0.10, feasibility);
    if (!Check(!edge.hard_feasible &&
                   edge.reject_reason ==
                       go2_terrain::FootholdRejectReason::kSurfaceStep,
               "step edge was accepted as one safe patch"))
        return 1;

    const go2::Vec3 swing_start{0.18, -0.10, -0.25};
    const go2::Vec3 swing_end{0.28, -0.10, -0.25};
    double clearance = 0.0;
    double required_lift = 0.0;
    if (!Check(go2_terrain::CheckSwingClearance(
                   built.model, swing_start, swing_end, 0.03, clearance,
                   nullptr, go2::Leg::FR, &required_lift),
               "valid swept swing was rejected") ||
        !Check(required_lift >= 0.03,
               "swept clearance did not report required lift") ||
        !Check(!go2_terrain::CheckSwingClearance(
                   built.model, {0.18, -0.10, -0.32},
                   {0.28, -0.10, -0.32}, 0.03, clearance),
               "low swing was accepted"))
        return 1;

    double step_clearance = 0.0;
    double step_required_lift = 0.0;
    double step_peak_phase = 0.0;
    double step_leading_edge_phase = 0.0;
    bool step_leading_edge_phase_valid = false;
    go2_terrain::FootholdRejectReason step_swing_reason =
        go2_terrain::FootholdRejectReason::kNone;
    if (!Check(go2_terrain::CheckSwingClearance(
                   step_built.model, {0.18, -0.10, -0.25},
                   {0.425, -0.10, -0.15}, 0.03, step_clearance,
                   &step_swing_reason, go2::Leg::FL, &step_required_lift,
                   &step_peak_phase, &step_leading_edge_phase,
                   &step_leading_edge_phase_valid),
               "10cm sensor-derived swing clearance was rejected") ||
        !Check(step_required_lift >= 0.03 &&
                   step_required_lift < 0.30 &&
                   step_peak_phase >= 0.10 && step_peak_phase <= 0.90 &&
                   step_leading_edge_phase_valid &&
                   step_leading_edge_phase >= 0.10 &&
                   step_leading_edge_phase <= 0.75 &&
                   go2_terrain::TerrainSwingPathProgress(
                       step_leading_edge_phase, true,
                       step_leading_edge_phase) > 0.10 &&
                   go2_terrain::TerrainSwingPathProgress(
                       1.0, true, step_leading_edge_phase) > 0.999,
               "10cm swing clearance geometry was invalid"))
        return 1;

    // epoch15a/16 anchor false positive: the FK support foot stands on
    // flat ground at the riser base, but its own 5 cm cell (first riser
    // column, x in [0.30,0.35)) is filled with the riser top (-0.15),
    // reading ~39 mm above the measured foot.  The anchor check must
    // compare against the neighborhood minimum (the flat -0.25 the foot
    // actually stands on), while a start below even the lowest nearby
    // cell must still reject.
    {
        double anchor_clearance = 0.0;
        if (!Check(go2_terrain::CheckSwingClearance(
                       step_built.model, {0.32, -0.10, -0.1887},
                       {0.52, -0.10, -0.15}, 0.03, anchor_clearance,
                       nullptr, go2::Leg::FL),
                   "support anchor in a riser-filled cell was rejected") ||
            !Check(!go2_terrain::CheckSwingClearance(
                       step_built.model, {0.32, -0.10, -0.30},
                       {0.52, -0.10, -0.15}, 0.03, anchor_clearance),
                   "anchor below every nearby cell was accepted"))
            return 1;
    }

    // epoch18: CoversPatch distinguishes "no observation at all" (patch
    // crosses the grid boundary) from observed-but-unknown cells.
    {
        const auto &m = built.model;  // 24x20, res 0.05, origin (-0.5,-0.5)
        if (!Check(m.CoversPatch(0.0, 0.0, 0.025), "interior patch not covered") ||
            !Check(!m.CoversPatch(-0.49, 0.0, 0.025),
                   "boundary-crossing patch reported covered") ||
            !Check(!m.CoversPatch(-0.60, 0.0, 0.025),
                   "out-of-grid patch reported covered"))
            return 1;
    }

    // epoch18: the real local window is narrow in y (+/-0.225 m) and a
    // rear leg's knee/shin grazes that unobserved fringe in every swing.
    // Fringe shin samples must be skipped (no observation, no
    // constraint), not rejected with kUnknown — epoch17 starved all 32
    // rear-leg regions at the crux on this while the window was fully
    // known.
    {
        unitree_go::msg::dds_::HeightMap_ narrow_map;
        narrow_map.stamp(10.0);
        narrow_map.frame_id("base_link");
        narrow_map.resolution(0.05f);
        narrow_map.width(32);
        narrow_map.height(10);
        narrow_map.origin() = {-0.45f, -0.225f};
        narrow_map.data().assign(
            static_cast<std::size_t>(narrow_map.width()) *
                narrow_map.height(),
            -0.30f);
        const auto narrow_built = go2_terrain::BuildTerrainModel(
            &narrow_map, 10.04, 6, go2_terrain::TerrainSource::kLidar);
        double narrow_clearance = 0.0;
        go2_terrain::FootholdRejectReason narrow_reason =
            go2_terrain::FootholdRejectReason::kNone;
        if (!Check(narrow_built.ok(), "narrow map did not build") ||
            !Check(go2_terrain::CheckSwingClearance(
                       narrow_built.model, {-0.20, -0.156, -0.30},
                       {0.10, -0.156, -0.30}, 0.03, narrow_clearance,
                       &narrow_reason, go2::Leg::RR) &&
                   narrow_reason == go2_terrain::FootholdRejectReason::kNone,
                   "narrow-window rear-leg swing rejected at the fringe"))
            return 1;

        // An anchor drifted onto the lateral FOV fringe (y=-0.21, patch
        // crosses the window edge, every in-grid cell known — the epoch17
        // crux slip geometry) must use the observed subset, while a
        // genuine occlusion hole inside the window must still reject.
        go2_terrain::FootholdRejectReason fringe_reason =
            go2_terrain::FootholdRejectReason::kNone;
        if (!Check(go2_terrain::CheckSwingClearance(
                       narrow_built.model, {-0.20, -0.21, -0.30},
                       {-0.02, -0.17, -0.30}, 0.03, narrow_clearance,
                       &fringe_reason, go2::Leg::RR) &&
                   fringe_reason == go2_terrain::FootholdRejectReason::kNone,
                   "fringe-anchor swing rejected on FOV-edge cells"))
            return 1;
        auto hole_map = narrow_map;
        for (const std::size_t idx :
                 {0 * 32 + 6, 0 * 32 + 7, 1 * 32 + 6, 1 * 32 + 7})
            hole_map.data()[idx] = std::numeric_limits<float>::quiet_NaN();
        const auto hole_built = go2_terrain::BuildTerrainModel(
            &hole_map, 10.04, 7, go2_terrain::TerrainSource::kLidar);
        go2_terrain::FootholdRejectReason hole_reason =
            go2_terrain::FootholdRejectReason::kNone;
        if (!Check(hole_built.ok(), "hole map did not build") ||
            !Check(!go2_terrain::CheckSwingClearance(
                       hole_built.model, {-0.20, -0.21, -0.30},
                       {-0.02, -0.17, -0.30}, 0.03, narrow_clearance,
                       &hole_reason, go2::Leg::RR) &&
                   hole_reason == go2_terrain::FootholdRejectReason::kUnknown,
                   "in-window occlusion hole did not reject"))
            return 1;
    }

    // The default swing-speed cap must predict the measured crux step-up
    // swing (162 ms realized over an ~0.41 m L1 path) instead of doubling
    // it: the old 2.50 m/s cap stretched the timeline ~10 knots past
    // nominal and exhausted the two-contact drift band mid-crossing.
    // Flat nominal swings must stay nominal.
    {
        const go2_terrain::TerrainFeasibilityConfig default_feasibility;
        const double crux_duration = go2_terrain::TerrainSwingDurationForPath(
            0.125, {0.45, 0.13, 0.0}, {0.72, 0.13, 0.05}, 0.09,
            default_feasibility.max_swing_speed_mps);
        const double flat_duration = go2_terrain::TerrainSwingDurationForPath(
            0.125, {0.20, 0.13, 0.0}, {0.35, 0.13, 0.0}, 0.08,
            default_feasibility.max_swing_speed_mps);
        if (!Check(crux_duration >= 0.125 && crux_duration <= 0.195,
                   "default swing-speed cap mispredicts the crux swing") ||
            !Check(std::abs(flat_duration - 0.125) < 1.0e-9,
                   "default swing-speed cap stretches flat swings"))
            return 1;
    }


    auto high_step_map = map;
    for (std::size_t iy = 0; iy < high_step_map.height(); ++iy)
    {
        for (std::size_t ix = 14; ix < high_step_map.width(); ++ix)
            high_step_map.data()[iy * high_step_map.width() + ix] = -0.10f;
    }
    const auto high_step_built = go2_terrain::BuildTerrainModel(
        &high_step_map, 10.04, 4, go2_terrain::TerrainSource::kLidar);
    auto repeated_step_map = step_map;
    for (std::size_t iy = 0; iy < repeated_step_map.height(); ++iy)
    {
        for (std::size_t ix = 20; ix < repeated_step_map.width(); ++ix)
            repeated_step_map.data()[iy * repeated_step_map.width() + ix] =
                -0.10f;
    }
    const auto repeated_step_built = go2_terrain::BuildTerrainModel(
        &repeated_step_map, 10.04, 5, go2_terrain::TerrainSource::kLidar);
    double high_step_clearance = 0.0;
    double high_step_required_lift = 0.0;
    if (!Check(go2_terrain::CheckSwingClearance(
                   high_step_built.model, {0.18, -0.10, -0.25},
                   {0.425, -0.10, -0.10}, 0.03, high_step_clearance,
                   nullptr, go2::Leg::FL, &high_step_required_lift),
               "15cm sensor-derived swing clearance was rejected") ||
        !Check(high_step_required_lift >= 0.03 &&
                   high_step_required_lift < 0.40,
               "15cm swing clearance geometry was invalid"))
        return 1;

    go2_terrain::TerrainPlannerInput input;
    input.terrain = &built.model;
    input.state_stamp_s = 10.04;
    input.base_position_world = {0.0, 0.0, 0.0};
    input.base_height_m = 0.42;
    input.contact_schedule.measured_contact = {true, false, false, true};
    input.contact_schedule.measured_valid = true;
    input.current_feet_base = {
        go2::Vec3{0.20, -0.10, -0.25},
        go2::Vec3{0.20, 0.10, -0.25},
        go2::Vec3{-0.20, -0.10, -0.25},
        go2::Vec3{-0.20, 0.10, -0.25}};
    input.nominal_feet_base = input.current_feet_base;
    for (std::size_t k = 0; k < 8; ++k)
    {
        input.contact_schedule.planned_contact[k] = {true, false, false, true};
        if (k >= 2)
            input.contact_schedule.planned_contact[k] = {true, true, true, true};
    }
    input.contact_schedule.planned_valid = true;
    go2_terrain::TerrainPlannerConfig planner_config;
    planner_config.sensor_only = true;
    planner_config.allow_actuation = false;
    go2_terrain::TerrainPlanner planner(planner_config);
    const auto planned = planner.Build(input, 7);
    if (!Check(!planned.publishable &&
                   planned.plan.status ==
                       go2_terrain::TerrainPlanStatus::kDegraded,
               "sensor-only planner became actuation-capable") ||
        !Check(planned.candidate_counts[0] > 0 &&
                   planned.candidate_counts[1] > 0 &&
                   planned.candidate_counts[2] > 0 &&
                   planned.candidate_counts[3] > 0 &&
                   !planned.selected[1].hard_feasible,
               "sensor-only planner performed actuation selection"))
        return 1;

    planner_config.sensor_only = false;
    planner_config.allow_actuation = true;
    go2_terrain::TerrainPlanner actuation_planner(planner_config);

    auto forward_step_input = input;
    forward_step_input.terrain = &step_built.model;
    forward_step_input.commanded_vx_mps = 0.30;
    forward_step_input.next_touchdown_time_valid.fill(false);
    const auto forward_step_plan = actuation_planner.Build(
        forward_step_input, 12);
    if (!Check(forward_step_plan.publishable &&
                   forward_step_plan.selected[1].hard_feasible &&
                   forward_step_plan.selected[1].foot_position.z > -0.20,
               "forward sensor-elevated foothold was not selected"))
        return 1;
    bool transition_marked = false;
    for (std::size_t k = 0; k < go2_terrain::kTerrainPlanMaxKnots; ++k)
        transition_marked = transition_marked ||
            forward_step_plan.plan.predicted_foothold[k][1]
                .surface_transition_required;
    if (!Check(transition_marked,
               "sensor-elevated foothold did not mark a surface transition") ||
        !Check(!forward_step_plan.plan.velocity_request.valid,
               "sensor-elevated foothold injected a nominal velocity request"))
        return 1;

    auto contact_gap_input = forward_step_input;
    contact_gap_input.terrain_retarget_allowed_valid = true;
    contact_gap_input.terrain_retarget_allowed.fill(true);
    contact_gap_input.contact_schedule.measured_contact[0] = false;
    const auto contact_gap_plan = actuation_planner.Build(
        contact_gap_input, 15);
    if (!Check(contact_gap_plan.candidate_required[0] &&
                   contact_gap_plan.selected[0].hard_feasible &&
                   contact_gap_plan.selected[0].foot_position.z > -0.20,
               "contact-gap front leg was not replanned from sensor terrain"))
        return 1;
    // Three legs measured airborne cannot be made feasible by any retime;
    // the plan must remain rejected.  The timeline-stretch retime truncates
    // horizon overflows gracefully, but a truncated retime over a measured
    // state with fewer than two contacts fails closed (kNoSafeFoothold)
    // before the geometric support check (kSupportInfeasible) — either is a
    // valid fail-closed rejection.
    if (!Check(!contact_gap_plan.publishable &&
                   (contact_gap_plan.plan.failure ==
                        go2_terrain::TerrainPlanFailure::kSupportInfeasible ||
                    contact_gap_plan.plan.failure ==
                        go2_terrain::TerrainPlanFailure::kNoSafeFoothold),
               "contact-gap plan was not rejected fail-closed"))
        return 1;
    if (!Check(forward_step_plan.plan.body_reference[20].position.z <=
                   forward_step_plan.plan.body_reference[0].position.z +
                       1.0e-4,
               "terrain body reference rose before measured touchdown"))
        return 1;

    auto confirmed_step_input = forward_step_input;
    confirmed_step_input.terrain_surface_transition_active = true;
    confirmed_step_input.terrain_surface_transition_required.fill(false);
    confirmed_step_input.terrain_surface_transition_required[1] = true;
    confirmed_step_input.terrain_surface_transition_committed.fill(false);
    confirmed_step_input.terrain_surface_transition_committed[1] = true;
    const auto confirmed_step_plan = actuation_planner.Build(
        confirmed_step_input, 13);
    if (!Check(confirmed_step_plan.publishable &&
                   confirmed_step_plan.plan.body_reference[20].position.z >
                       confirmed_step_plan.plan.body_reference[0].position.z +
                           1.0e-4,
               "confirmed terrain surface did not raise body reference"))
        return 1;

    const auto actuation_plan = actuation_planner.Build(input, 8);
    if (!Check(actuation_plan.publishable && actuation_plan.plan.valid(),
               "actuation planner did not publish a valid plan") ||
        !Check(actuation_plan.plan.committed_touchdowns > 0,
               "valid planner did not commit a touchdown") ||
        !Check(std::isfinite(actuation_plan.plan.min_edge_margin_m) &&
                   std::isfinite(actuation_plan.plan.min_support_margin_m),
               "planner validity metrics are not finite") ||
        !Check(std::all_of(
                   actuation_plan.plan.current_terrain_height_valid.begin(),
                   actuation_plan.plan.current_terrain_height_valid.end(),
                   [](bool valid) { return valid; }),
               "planner did not preserve per-leg sensor terrain heights") ||
        !Check(actuation_plan.plan.current_support_surface_valid[0] &&
                   actuation_plan.plan.current_support_surface_valid[3] &&
                   !actuation_plan.plan.current_support_surface_valid[1] &&
                   !actuation_plan.plan.current_support_surface_valid[2],
               "sensor terrain height was promoted to measured support") ||
        !Check(actuation_plan.plan.body_reference[0].yaw_rad == 0.0,
               "planner did not preserve body yaw reference") ||
        !Check(std::abs(
                   actuation_plan.plan.body_reference[7].position.z -
                   actuation_plan.plan.body_reference[0].position.z) <
                   1.0e-9,
               "flat terrain changed the body height reference") ||
        !Check(actuation_plan.selected[1].region_id <
                   actuation_plan.regions[1].size(),
               "actuation planner did not consume a safe region") ||
        !Check(std::abs(actuation_plan.selected[1].foot_position.x -
                            input.nominal_feet_base[1].x) < 1.0e-9 &&
                   std::abs(actuation_plan.selected[1].foot_position.y -
                            input.nominal_feet_base[1].y) < 1.0e-9,
               "flat safe region displaced the nominal foothold") ||
        !Check(actuation_plan.selected[1].swing_lift_m >= 0.03,
               "actuation planner did not propagate swept lift"))
        return 1;

    auto moving_body_input = input;
    moving_body_input.base_velocity_world = {0.25, 0.0, 0.0};
    const auto moving_body_plan = actuation_planner.Build(
        moving_body_input, 13);
    if (!Check(moving_body_plan.publishable &&
                   moving_body_plan.plan.body_reference[4].position.x >
                       moving_body_plan.plan.body_reference[0].position.x,
               "planner did not advance its future body reference") ||
        !Check(moving_body_plan.plan.body_reference[4].position.x > 0.0,
               "future body reference did not use measured velocity"))
        return 1;

    // An armed terrain swing and the complete SRBD preview must remain one
    // atomic transaction after consumer latency.  Short storage makes gait
    // retain the touchdown while WBC drops its future contact schedule.
    auto execution_config = planner_config;
    execution_config.plan_validity_s = 0.50;
    go2_terrain::TerrainPlanner execution_planner(execution_config);
    const auto execution_plan = execution_planner.Build(
        forward_step_input, 16);
    std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
        execution_indices{};
    if (!Check(execution_planner.config().horizon_knots == 24,
               "execution tail enlarged the optimization horizon") ||
        !Check(execution_plan.publishable && execution_plan.plan.valid(),
               "execution-lifetime plan was not publishable") ||
        !Check(execution_plan.plan.horizon_knots ==
                   go2_terrain::kTerrainPlanMaxKnots,
               "execution support tail was not stored") ||
        !Check(go2_terrain::BuildTerrainPlanHorizonIndices(
                   execution_plan.plan,
                   execution_plan.plan.state_stamp_s + 0.10,
                   execution_config.knot_dt_s, 0.05, 8,
                   execution_indices),
               "atomic plan did not cover the delayed SRBD horizon"))
        return 1;

    auto measured_support_input = input;
    auto repeated_input = forward_step_input;
    repeated_input.terrain = &repeated_step_built.model;
    // Keep this synthetic repeated-contact schedule coherent with the
    // planner's terrain-conditioned swing duration: the first event is
    // deliberately retimed, while the second event remains inside the
    // finite preview horizon.
    repeated_input.gait_period_s = 0.10;
    repeated_input.base_velocity_world = {0.0, 0.0, 0.0};
    repeated_input.contact_schedule.measured_contact =
        {true, false, false, true};
    for (std::size_t k = 0; k < 24; ++k)
    {
        if (k < 2)
            repeated_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else if (k < 6)
            repeated_input.contact_schedule.planned_contact[k] =
                {true, true, true, true};
        else if (k < 10)
            repeated_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else
            repeated_input.contact_schedule.planned_contact[k] =
                {true, true, true, true};
    }
    repeated_input.contact_schedule.planned_valid = true;
    auto repeated_planner_config = planner_config;
    repeated_planner_config.plan_validity_s = 0.50;
    repeated_planner_config.feasibility.max_swing_speed_mps = 10.0;
    go2_terrain::TerrainPlanner repeated_planner(
        repeated_planner_config);
    const auto repeated_plan = repeated_planner.Build(
        repeated_input, 14);
    int first_repeated_touchdown = -1;
    int second_repeated_touchdown = -1;
    for (std::size_t k = 0; k < 24; ++k)
    {
        if (!repeated_plan.plan.predicted_foothold[k][1].touchdown)
            continue;
        if (first_repeated_touchdown < 0)
            first_repeated_touchdown = static_cast<int>(k);
        else if (second_repeated_touchdown < 0)
            second_repeated_touchdown = static_cast<int>(k);
    }
    if (!Check(repeated_plan.publishable &&
                   repeated_plan.plan.valid(),
               "repeated terrain plan was not publishable") ||
        !Check(first_repeated_touchdown >= 0 &&
                   second_repeated_touchdown > first_repeated_touchdown,
               "plan did not preserve repeated touchdown events") ||
        !Check(repeated_plan.plan.predicted_foothold[
                   static_cast<std::size_t>(second_repeated_touchdown)][1].valid &&
                   repeated_plan.plan.predicted_foothold[
                       static_cast<std::size_t>(second_repeated_touchdown)][1].position_world.x >
                   repeated_plan.plan.predicted_foothold[
                       static_cast<std::size_t>(first_repeated_touchdown)][1].position_world.x,
               "future touchdown did not advance its sensor foothold") ||
        !Check(repeated_plan.plan.predicted_foothold[
                   static_cast<std::size_t>(second_repeated_touchdown)][1].
                       swing_start_position_valid &&
                   std::abs(repeated_plan.plan.predicted_foothold[
                       static_cast<std::size_t>(second_repeated_touchdown)][1].
                       swing_start_position_world.x -
                       repeated_plan.plan.predicted_foothold[
                           static_cast<std::size_t>(first_repeated_touchdown)][1].
                           position_world.x) < 1.0e-9,
               "future touchdown lost its checked swing start"))
        return 1;

    // A terrain retime whose delay pushes a late touchdown event past the
    // horizon must degrade gracefully — truncate the stretch at the horizon
    // end, drop the far event, and still publish the near-term schedule —
    // instead of rejecting the whole plan with kNoSafeFoothold, as long as
    // the measured state is independently supported (a two-contact diagonal
    // here; the under-supported fail-closed case is pinned by the
    // contact-gap check above).
    auto overflow_input = repeated_input;
    for (std::size_t k = 0; k < 24; ++k)
    {
        if (k < 23)
            overflow_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else
            overflow_input.contact_schedule.planned_contact[k] =
                {true, true, true, true};
    }
    const auto overflow_plan = repeated_planner.Build(overflow_input, 15);
    if (!Check(overflow_plan.publishable && overflow_plan.plan.valid(),
               "horizon-overflow retime was not truncated gracefully") ||
        !Check(overflow_plan.plan.failure !=
                   go2_terrain::TerrainPlanFailure::kNoSafeFoothold,
               "horizon-overflow retime still rejected the plan") ||
        !Check(!overflow_plan.plan.predicted_foothold[23][1].touchdown,
               "overflow retime left the touchdown event unshifted"))
        return 1;
    for (std::size_t k = 0; k < 16; ++k)
    {
        if (!Check(overflow_plan.plan.contact_schedule.planned_contact[k] ==
                       overflow_input.contact_schedule.planned_contact[k],
                   "graceful truncation changed the near-term schedule"))
            return 1;
    }

    measured_support_input.contact_schedule.measured_contact =
        {true, true, true, true};
    measured_support_input.current_feet_base = {
        go2::Vec3{0.214, -0.135, -0.25},
        go2::Vec3{0.214, 0.135, -0.25},
        go2::Vec3{-0.174, -0.193, -0.25},
        go2::Vec3{-0.174, 0.193, -0.25}};
    measured_support_input.nominal_feet_base =
        measured_support_input.current_feet_base;
    for (std::size_t k = 0; k < 8; ++k)
        measured_support_input.contact_schedule.planned_contact[k] =
            {true, false, false, true};
    const auto measured_support_plan =
        actuation_planner.Build(measured_support_input, 11);
    if (!Check(measured_support_plan.publishable &&
                   measured_support_plan.plan.valid() &&
                   measured_support_plan.plan.current_support_count == 4,
               "measured support geometry was rejected as planned diagonal"))
        return 1;

    auto flight_input = input;
    for (std::size_t k = 0; k < 8; ++k)
    {
        if (k < 2)
            flight_input.contact_schedule.planned_contact[k] =
                {true, false, false, true};
        else if (k < 4)
            flight_input.contact_schedule.planned_contact[k] =
                {false, false, false, false};
        else
            flight_input.contact_schedule.planned_contact[k] =
                {false, true, true, false};
    }
    const auto flight_plan = actuation_planner.Build(flight_input, 9);
    if (!Check(flight_plan.publishable && flight_plan.plan.valid(),
               "running-trot flight knot was rejected") ||
        !Check(flight_plan.plan.committed_touchdowns > 0,
               "flight schedule did not retain touchdown planning"))
        return 1;

    std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
        plan_indices{};
    if (!Check(go2_terrain::BuildTerrainPlanHorizonIndices(
                   flight_plan.plan, 10.08, 0.02, 0.02, 4,
                   plan_indices) &&
                   plan_indices[0] == 2 && plan_indices[3] == 5,
               "terrain plan horizon was not time-aligned") ||
        !Check(go2_terrain::BuildTerrainPlanHorizonIndices(
                   flight_plan.plan, 10.10, 0.02, 0.02, 12,
                   plan_indices) &&
                   plan_indices[0] == 3 && plan_indices[11] == 14,
               "extended terrain horizon was not time-aligned") ||
        !Check(!go2_terrain::BuildTerrainPlanHorizonIndices(
                   flight_plan.plan, 10.90, 0.02, 0.02, 12,
                   plan_indices),
               "plan beyond the bounded support tail was not rejected"))
        return 1;

    auto no_support_input = flight_input;
    for (auto &contact : no_support_input.contact_schedule.planned_contact)
        contact = {false, false, false, false};
    const auto no_support_plan = actuation_planner.Build(no_support_input, 10);
    if (!Check(!no_support_plan.publishable &&
                   no_support_plan.plan.failure ==
                       go2_terrain::TerrainPlanFailure::kSupportInfeasible,
               "support-free schedule was accepted"))
        return 1;

    const std::array<bool, go2::kLegCount> flight_contact{
        false, false, false, false};
    const std::array<bool, go2::kLegCount> held_diagonal{
        true, false, false, true};
    const std::array<bool, go2::kLegCount> active_left_front_target{
        false, true, false, false};
    const auto transfer_contact =
        go2_terrain::TerrainTransferPreviewContact(
            flight_contact, held_diagonal, active_left_front_target);
    if (!Check(transfer_contact[0] && !transfer_contact[1] &&
                   !transfer_contact[2] && transfer_contact[3],
               "active target removed an unrelated held support foot"))
        return 1;

    go2_terrain::TerrainMotionPlan atomic_plan;
    atomic_plan.plan_id = 1;
    atomic_plan.plan_epoch = 1;
    atomic_plan.map_epoch = 1;
    atomic_plan.state_stamp_s = 1.0;
    atomic_plan.generated_at_s = 1.0;
    atomic_plan.valid_until_s = 2.0;
    atomic_plan.frame_id = "base_link";
    atomic_plan.status = go2_terrain::TerrainPlanStatus::kValid;
    atomic_plan.horizon_knots = 1;
    atomic_plan.body_reference[0].valid = true;
    atomic_plan.contact_schedule = input.contact_schedule;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        atomic_plan.contact_schedule.planned_contact[0][leg] = true;
        atomic_plan.predicted_foothold[0][leg].valid = true;
        atomic_plan.predicted_foothold[0][leg].position_world =
            input.current_feet_base[leg];
    }
    go2_terrain::TerrainPlanStore store;
    store.Publish(atomic_plan);
    const auto loaded = store.LoadUsable(1.5);
    if (!Check(loaded && loaded->plan_id == 1 && loaded->map_epoch == 1,
               "terrain plan was not atomically published"))
        return 1;
    if (!Check(loaded->plan_epoch == 1 &&
                   loaded->contact_schedule.measured_contact[0] &&
                   !loaded->contact_schedule.measured_contact[1],
               "planned and measured contact state was not preserved"))
        return 1;

    // A terrain-conditioned swing that needs more time than the nominal
    // window must stretch the WHOLE contact timeline: the opposite support
    // pair stays loaded while the extended swing is airborne, and every
    // later event inherits the shift.  Regression guard for the deadlock
    // where only the risen leg's touchdown was delayed while the opposite
    // pair's swing still started on the raw phase, leaving knots with zero
    // contacts for gait/MPC/WBC to disagree about.
    auto stretch_input = forward_step_input;
    stretch_input.gait_period_s = 0.50;
    stretch_input.duty_factor = 0.75;
    stretch_input.base_velocity_world = {0.0, 0.0, 0.0};
    stretch_input.contact_schedule.measured_contact =
        {true, false, false, true};
    for (std::size_t k = 0; k < go2_terrain::kTerrainPlanMaxKnots; ++k)
    {
        stretch_input.contact_schedule.planned_contact[k] =
            ((k / 7) % 2 == 0)
                ? std::array<bool, go2::kLegCount>{true, false, false, true}
                : std::array<bool, go2::kLegCount>{false, true, true, false};
    }
    auto stretch_config = planner_config;
    stretch_config.feasibility.max_swing_speed_mps = 5.5;
    go2_terrain::TerrainPlanner stretch_planner(stretch_config);
    const auto stretched_plan = stretch_planner.Build(stretch_input, 22);
    bool stretched_two_contacts = stretched_plan.plan.valid();
    for (std::size_t k = 0;
         k < stretched_plan.plan.horizon_knots && stretched_two_contacts;
         ++k)
    {
        int contacts = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            contacts += stretched_plan.plan.contact_schedule
                .planned_contact[k][leg] ? 1 : 0;
        if (contacts < 2)
            stretched_two_contacts = false;
    }
    bool previous = true;
    int stretched_first_touchdown = -1;
    for (std::size_t k = 0; k < stretched_plan.plan.horizon_knots; ++k)
    {
        const bool contact = stretched_plan.plan.contact_schedule
            .planned_contact[k][1];
        if (contact && !previous)
        {
            stretched_first_touchdown = static_cast<int>(k);
            break;
        }
        previous = contact;
    }
    if (!Check(stretched_plan.publishable && stretched_plan.plan.valid(),
               "stretched terrain plan was not publishable") ||
        !Check(stretched_two_contacts,
               "stretched schedule dropped below two contacts") ||
        !Check(stretched_first_touchdown > 7,
               "terrain touchdown was not stretched beyond the nominal knot"))
        return 1;

    // A stretched touchdown on a forward-moving body lands after the base
    // has travelled past the foothold selected for the nominal event time.
    // The retime must carry the foothold forward with the stretch so the
    // base-relative landing geometry matches the stationary case.
    auto moving_stretch_input = stretch_input;
    moving_stretch_input.base_velocity_world = {0.30, 0.0, 0.0};
    go2_terrain::TerrainPlanner moving_stretch_planner(stretch_config);
    const auto moving_stretched_plan = moving_stretch_planner.Build(
        moving_stretch_input, 23);
    int nominal_touchdown_knot = -1;
    for (std::size_t k = 0;
         k < stretch_input.contact_schedule.planned_contact.size(); ++k)
    {
        if (stretch_input.contact_schedule.planned_contact[k][1])
        {
            nominal_touchdown_knot = static_cast<int>(k);
            break;
        }
    }
    bool moving_previous = true;
    int moving_first_touchdown = -1;
    for (std::size_t k = 0;
         k < moving_stretched_plan.plan.horizon_knots; ++k)
    {
        const bool contact = moving_stretched_plan.plan.contact_schedule
            .planned_contact[k][1];
        if (contact && !moving_previous)
        {
            moving_first_touchdown = static_cast<int>(k);
            break;
        }
        moving_previous = contact;
    }
    if (!Check(moving_stretched_plan.publishable &&
                   moving_stretched_plan.plan.valid(),
               "moving stretched terrain plan was not publishable") ||
        !Check(nominal_touchdown_knot > 0 &&
                   moving_first_touchdown > nominal_touchdown_knot,
               "moving terrain touchdown was not stretched") ||
        // A positive terrain delay reserves one additional 20 ms planner
        // knot before publication, leaving a discrete handoff margin for the
        // execution rebase rather than spending all delay on path duration.
        !Check(moving_first_touchdown >= nominal_touchdown_knot + 2,
               "terrain retime did not reserve the handoff margin knot"))
        return 1;
    const auto &moving_foot = moving_stretched_plan.plan.predicted_foothold[
        static_cast<std::size_t>(moving_first_touchdown)][1];
    const auto &static_foot = stretched_plan.plan.predicted_foothold[
        static_cast<std::size_t>(stretched_first_touchdown)][1];
    const double carry_forward_m = 0.30 *
        static_cast<double>(moving_first_touchdown - nominal_touchdown_knot) *
        stretch_config.knot_dt_s;
    if (!Check(moving_foot.valid && static_foot.valid,
               "stretched touchdown foothold was not populated") ||
        !Check(std::abs(moving_foot.position_world.x -
                            static_foot.position_world.x -
                            carry_forward_m) < 0.01,
               "stretched touchdown did not travel with the moving body"))
        return 1;

    std::cout << "Terrain model, feasibility, planner, and atomic plan checks passed.\n";
    return 0;
}
