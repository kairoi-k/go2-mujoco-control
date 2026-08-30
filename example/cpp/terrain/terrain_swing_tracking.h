#pragma once

namespace go2_terrain
{

struct TerrainSwingTrackingParameters
{
    double position_gain;
    double velocity_gain;
    double acceleration_limit;
};

// The crawl transfer uses the flat tracking gains so the raised swing does not
// inject a large wrench into the three-foot support handoff. The branch
// remains explicit so flat and transfer callers retain separate contracts.
constexpr TerrainSwingTrackingParameters TerrainSwingTrackingForTransfer(
    bool transfer_window_active)
{
    return transfer_window_active
        ? TerrainSwingTrackingParameters{180.0, 16.0, 50.0}
        : TerrainSwingTrackingParameters{180.0, 16.0, 50.0};
}

}  // namespace go2_terrain
