#pragma once

namespace go2_terrain
{

struct TerrainSwingTrackingParameters
{
    double position_gain;
    double velocity_gain;
    double acceleration_limit;
};

// The crawl transfer has a longer, slower endpoint trajectory than the
// nominal trot, but it still needs enough closed-loop authority to reject the
// body-motion lag seen at the immutable touchdown boundary. Flat-ground and
// non-transfer paths retain the established gains bit-for-bit.
constexpr TerrainSwingTrackingParameters TerrainSwingTrackingForTransfer(
    bool transfer_window_active)
{
    return transfer_window_active
        ? TerrainSwingTrackingParameters{180.0, 16.0, 50.0}
        : TerrainSwingTrackingParameters{180.0, 16.0, 50.0};
}

}  // namespace go2_terrain
