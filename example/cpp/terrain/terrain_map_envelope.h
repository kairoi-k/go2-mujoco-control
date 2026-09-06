#pragma once

// Versioned, bounded terrain-map transport and capture-pose registration.
// The external HeightMap IDL remains the legacy baseline observation.  Terrain
// actuation consumes only this envelope after strict decoding and registration.

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>
#include <utility>

#include <unitree/idl/go2/HeightMap_.hpp>

namespace go2_terrain
{

constexpr double kTerrainMapUnknown =
    std::numeric_limits<double>::quiet_NaN();
constexpr double kTerrainMapMaxAgeS = 0.20;
constexpr double kTerrainMapMinResolutionM = 1.0e-4;
constexpr double kTerrainMapMaxResolutionM = 10.0;
constexpr double kTerrainMapTimeToleranceS = 1.0e-6;
constexpr double kTerrainMapHeightToleranceM = 1.0e-6;
constexpr std::size_t kTerrainMapEnvelopeMaxBytes = 256U * 1024U;
constexpr std::size_t kTerrainMapEnvelopeMaxCells = 4096U;
constexpr std::size_t kTerrainMapEnvelopeMaxFrameBytes = 64U;
constexpr std::uint32_t kTerrainMapEnvelopeVersion = 1U;

struct TerrainMapEnvelope
{
    std::uint32_t version = kTerrainMapEnvelopeVersion;
    std::uint64_t sequence = 0;
    double map_stamp_s = kTerrainMapUnknown;
    std::string frame_id;
    double resolution_m = 0.0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::array<double, 2> origin_m{0.0, 0.0};
    std::array<double, 3> capture_position_world{
        kTerrainMapUnknown, kTerrainMapUnknown, kTerrainMapUnknown};
    double capture_yaw_rad = kTerrainMapUnknown;
    // A finite height must have a finite observation stamp.  Unknown cells
    // carry the explicit token U for both vectors in the wire format.
    std::vector<double> heights_m;
    std::vector<double> observation_stamp_s;
};

enum class TerrainMapCodecError : std::uint8_t
{
    kNone = 0,
    kEmpty,
    kTooLarge,
    kBadMagic,
    kBadField,
    kBadNumber,
    kBadShape,
    kBadCellPair,
    kNonFinitePose,
};

struct TerrainMapCodecResult
{
    TerrainMapEnvelope envelope;
    TerrainMapCodecError error = TerrainMapCodecError::kNone;

    bool ok() const { return error == TerrainMapCodecError::kNone; }
};

inline bool TerrainMapFinite(double value)
{
    return std::isfinite(value);
}

inline bool TerrainMapStrictDouble(std::string_view token, double &value)
{
    if (token.empty())
        return false;
    for (const char c : token)
    {
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
            return false;
    }
    std::istringstream input{std::string(token)};
    input.imbue(std::locale::classic());
    input >> std::noskipws;
    if (!(input >> value) || !std::isfinite(value))
        return false;
    char extra = 0;
    return !(input >> extra);
}

inline bool TerrainMapStrictUint64(std::string_view token,
                                   std::uint64_t &value)
{
    if (token.empty())
        return false;
    for (const char c : token)
    {
        if (c < '0' || c > '9')
            return false;
    }
    const char *begin = token.data();
    const char *end = begin + token.size();
    const auto parsed = std::from_chars(begin, end, value, 10);
    return parsed.ec == std::errc{} && parsed.ptr == end;
}

inline bool TerrainMapStrictUint32(std::string_view token,
                                   std::uint32_t &value)
{
    std::uint64_t wide = 0;
    if (!TerrainMapStrictUint64(token, wide) ||
        wide > std::numeric_limits<std::uint32_t>::max())
        return false;
    value = static_cast<std::uint32_t>(wide);
    return true;
}

inline bool TerrainMapValidFrame(std::string_view frame)
{
    if (frame.empty() || frame.size() > kTerrainMapEnvelopeMaxFrameBytes)
        return false;
    for (const char c : frame)
    {
        if (c == '=' || c == '\r' || c == '\n' || c == ' ' || c == '\t')
            return false;
    }
    return true;
}

inline bool TerrainMapEnvelopeShapeValid(const TerrainMapEnvelope &envelope)
{
    const std::size_t expected =
        static_cast<std::size_t>(envelope.width) * envelope.height;
    return envelope.version == kTerrainMapEnvelopeVersion &&
        envelope.sequence != 0 && std::isfinite(envelope.map_stamp_s) &&
        TerrainMapValidFrame(envelope.frame_id) &&
        std::isfinite(envelope.resolution_m) &&
        envelope.resolution_m >= kTerrainMapMinResolutionM &&
        envelope.resolution_m <= kTerrainMapMaxResolutionM &&
        envelope.width > 0 && envelope.height > 0 &&
        expected <= kTerrainMapEnvelopeMaxCells &&
        envelope.heights_m.size() == expected &&
        envelope.observation_stamp_s.size() == expected &&
        std::isfinite(envelope.origin_m[0]) &&
        std::isfinite(envelope.origin_m[1]) &&
        std::isfinite(envelope.capture_position_world[0]) &&
        std::isfinite(envelope.capture_position_world[1]) &&
        std::isfinite(envelope.capture_position_world[2]) &&
        std::isfinite(envelope.capture_yaw_rad);
}

template <typename Function>
inline bool TerrainMapParseList(std::string_view value, std::size_t count,
                                Function &&function)
{
    std::size_t start = 0;
    for (std::size_t index = 0; index < count; ++index)
    {
        const std::size_t comma = value.find(',', start);
        const std::size_t end = comma == std::string_view::npos
            ? value.size() : comma;
        if (end == start || !function(index, value.substr(start, end - start)))
            return false;
        if (comma == std::string_view::npos)
            return index + 1 == count;
        if (index + 1 == count)
            return false;
        start = comma + 1;
    }
    return false;
}

inline bool TerrainMapField(std::string_view line, std::string_view key,
                            std::string_view &value)
{
    const std::string prefix = std::string(key) + "=";
    if (line.size() <= prefix.size() || line.substr(0, prefix.size()) != prefix)
        return false;
    value = line.substr(prefix.size());
    return value.find('=') == std::string_view::npos;
}

inline TerrainMapCodecResult DeserializeTerrainMapEnvelope(
    std::string_view wire)
{
    TerrainMapCodecResult result;
    if (wire.empty()) { result.error = TerrainMapCodecError::kEmpty; return result; }
    if (wire.size() > kTerrainMapEnvelopeMaxBytes)
    { result.error = TerrainMapCodecError::kTooLarge; return result; }
    std::istringstream lines{std::string(wire)};
    lines.imbue(std::locale::classic());
    std::array<std::string, 15> line{};
    for (std::size_t i = 0; i < line.size(); ++i)
    {
        if (!std::getline(lines, line[i]) || line[i].empty() ||
            line[i].back() == '\r')
        { result.error = TerrainMapCodecError::kBadField; return result; }
    }
    std::string extra;
    if (std::getline(lines, extra) || line[0] != "GO2_TERRAIN_MAP_ENVELOPE_V1")
    { result.error = line[0] == "GO2_TERRAIN_MAP_ENVELOPE_V1"
          ? TerrainMapCodecError::kBadField : TerrainMapCodecError::kBadMagic; return result; }
    auto get = [&](std::size_t i, std::string_view key, std::string_view &v) {
        return TerrainMapField(line[i], key, v);
    };
    std::string_view v;
    TerrainMapEnvelope e;
    if (!get(1, "sequence", v) || !TerrainMapStrictUint64(v, e.sequence) ||
        e.sequence == 0 || !get(2, "map_stamp", v) ||
        !TerrainMapStrictDouble(v, e.map_stamp_s) ||
        !get(3, "frame", v) || !TerrainMapValidFrame(v))
    { result.error = TerrainMapCodecError::kBadField; return result; }
    e.frame_id = std::string(v);
    if (!get(4, "resolution", v) || !TerrainMapStrictDouble(v, e.resolution_m) ||
        !(e.resolution_m >= kTerrainMapMinResolutionM) ||
        !(e.resolution_m <= kTerrainMapMaxResolutionM) || !get(5, "width", v) ||
        !TerrainMapStrictUint32(v, e.width) || !get(6, "height", v) ||
        !TerrainMapStrictUint32(v, e.height) || e.width == 0 || e.height == 0)
    { result.error = TerrainMapCodecError::kBadShape; return result; }
    const std::size_t count = static_cast<std::size_t>(e.width) * e.height;
    if (count == 0 || count > kTerrainMapEnvelopeMaxCells)
    { result.error = TerrainMapCodecError::kBadShape; return result; }
    if (!get(7, "origin_x", v) || !TerrainMapStrictDouble(v, e.origin_m[0]) ||
        !get(8, "origin_y", v) || !TerrainMapStrictDouble(v, e.origin_m[1]) ||
        !get(9, "capture_x", v) ||
        !TerrainMapStrictDouble(v, e.capture_position_world[0]) ||
        !get(10, "capture_y", v) ||
        !TerrainMapStrictDouble(v, e.capture_position_world[1]) ||
        !get(11, "capture_z", v) ||
        !TerrainMapStrictDouble(v, e.capture_position_world[2]) ||
        !get(12, "capture_yaw", v) ||
        !TerrainMapStrictDouble(v, e.capture_yaw_rad))
    { result.error = TerrainMapCodecError::kNonFinitePose; return result; }
    std::string_view heights, stamps;
    if (!get(13, "heights", heights) || !get(14, "observation_stamps", stamps))
    { result.error = TerrainMapCodecError::kBadField; return result; }
    e.heights_m.assign(count, kTerrainMapUnknown);
    e.observation_stamp_s.assign(count, kTerrainMapUnknown);
    const bool heights_ok = TerrainMapParseList(
        heights, count, [&](std::size_t i, std::string_view token) {
            if (token == "U") return true;
            return TerrainMapStrictDouble(token, e.heights_m[i]);
        });
    const bool stamps_ok = TerrainMapParseList(
        stamps, count, [&](std::size_t i, std::string_view token) {
            if (token == "U") return true;
            return TerrainMapStrictDouble(token, e.observation_stamp_s[i]);
        });
    if (!heights_ok || !stamps_ok)
    { result.error = TerrainMapCodecError::kBadCellPair; return result; }
    for (std::size_t i = 0; i < count; ++i)
    {
        const bool have_height = std::isfinite(e.heights_m[i]);
        const bool have_stamp = std::isfinite(e.observation_stamp_s[i]);
        if (have_height != have_stamp)
        { result.error = TerrainMapCodecError::kBadCellPair; return result; }
    }
    if (!TerrainMapEnvelopeShapeValid(e))
    { result.error = TerrainMapCodecError::kNonFinitePose; return result; }
    result.envelope = std::move(e);
    return result;
}

inline bool SerializeTerrainMapEnvelope(const TerrainMapEnvelope &e,
                                        std::string &wire)
{
    if (!TerrainMapEnvelopeShapeValid(e))
        return false;
    const std::size_t count = e.heights_m.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        if (std::isfinite(e.heights_m[i]) !=
                std::isfinite(e.observation_stamp_s[i]))
            return false;
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "GO2_TERRAIN_MAP_ENVELOPE_V1\n"
        << "sequence=" << e.sequence << "\n"
        << "map_stamp=" << e.map_stamp_s << "\n"
        << "frame=" << e.frame_id << "\n"
        << "resolution=" << e.resolution_m << "\n"
        << "width=" << e.width << "\n"
        << "height=" << e.height << "\n"
        << "origin_x=" << e.origin_m[0] << "\n"
        << "origin_y=" << e.origin_m[1] << "\n"
        << "capture_x=" << e.capture_position_world[0] << "\n"
        << "capture_y=" << e.capture_position_world[1] << "\n"
        << "capture_z=" << e.capture_position_world[2] << "\n"
        << "capture_yaw=" << e.capture_yaw_rad << "\n"
        << "heights=";
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i != 0) out << ',';
        if (std::isfinite(e.heights_m[i])) out << e.heights_m[i];
        else out << 'U';
    }
    out << "\nobservation_stamps=";
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i != 0) out << ',';
        if (std::isfinite(e.observation_stamp_s[i]))
            out << e.observation_stamp_s[i];
        else out << 'U';
    }
    wire = out.str();
    return wire.size() <= kTerrainMapEnvelopeMaxBytes;
}

inline bool TerrainMapEnvelopeFromHeightMap(
    const unitree_go::msg::dds_::HeightMap_ &map, std::uint64_t sequence,
    const std::array<double, 3> &capture_position_world,
    double capture_yaw_rad, const std::vector<double> &observation_stamp_s,
    TerrainMapEnvelope &out)
{
    if (map.width() == 0 || map.height() == 0 ||
        map.data().size() != static_cast<std::size_t>(map.width()) * map.height() ||
        observation_stamp_s.size() != map.data().size())
        return false;
    out = {};
    out.sequence = sequence;
    out.map_stamp_s = map.stamp();
    out.frame_id = map.frame_id();
    out.resolution_m = map.resolution();
    out.width = map.width();
    out.height = map.height();
    out.origin_m = {map.origin()[0], map.origin()[1]};
    out.capture_position_world = capture_position_world;
    out.capture_yaw_rad = capture_yaw_rad;
    out.heights_m.resize(map.data().size(), kTerrainMapUnknown);
    out.observation_stamp_s = observation_stamp_s;
    for (std::size_t i = 0; i < map.data().size(); ++i)
    {
        if (std::isfinite(map.data()[i]))
        {
            if (!std::isfinite(observation_stamp_s[i])) return false;
            out.heights_m[i] = map.data()[i];
        }
        else if (std::isfinite(observation_stamp_s[i]))
            return false;
    }
    return TerrainMapEnvelopeShapeValid(out);
}

enum class TerrainMapRegistrationPolicy : std::uint8_t
{
    kLegacyScalarV1 = 0,
    kRegisteredIntervalsV2 = 1,
};

struct RegisteredTerrainMap
{
    unitree_go::msg::dds_::HeightMap_ map{};
    std::vector<double> cell_age_s;
    // V2 preserves a conservative interval for each registered destination cell.
    // V1 leaves these vectors empty and retains scalar map behavior.
    std::vector<double> cell_min_height_m;
    std::vector<double> cell_max_height_m;
    TerrainMapRegistrationPolicy registration_policy =
        TerrainMapRegistrationPolicy::kLegacyScalarV1;
    std::uint64_t sequence = 0;
    double map_stamp_s = kTerrainMapUnknown;
    double registration_state_stamp_s = kTerrainMapUnknown;
    std::array<double, 3> capture_position_world{
        kTerrainMapUnknown, kTerrainMapUnknown, kTerrainMapUnknown};
    double capture_yaw_rad = kTerrainMapUnknown;
    std::array<double, 3> current_position_world{
        kTerrainMapUnknown, kTerrainMapUnknown, kTerrainMapUnknown};
    double current_yaw_rad = kTerrainMapUnknown;
    bool registered = false;
};

enum class TerrainMapRegistrationError : std::uint8_t
{
    kNone = 0,
    kInvalidEnvelope,
    kFutureMap,
    kStaleMap,
    kInvalidStatePose,
    kInvalidCellAge,
    kStateStampMismatch,
};

struct TerrainMapRegistrationResult
{
    RegisteredTerrainMap map;
    TerrainMapRegistrationError error = TerrainMapRegistrationError::kNone;

    bool ok() const { return error == TerrainMapRegistrationError::kNone; }
};

struct TerrainMapXY
{
    double x = 0.0;
    double y = 0.0;
};

inline TerrainMapXY TerrainMapRotate(double yaw, TerrainMapXY p)
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * p.x - s * p.y, s * p.x + c * p.y};
}

inline TerrainMapXY TerrainMapToWorld(
    const std::array<double, 3> &base, double yaw, TerrainMapXY local)
{
    const TerrainMapXY rotated = TerrainMapRotate(yaw, local);
    return {base[0] + rotated.x, base[1] + rotated.y};
}

inline TerrainMapXY TerrainMapToCaptureLocal(
    const TerrainMapEnvelope &source, const std::array<double, 3> &current,
    double current_yaw, TerrainMapXY current_local)
{
    const TerrainMapXY world = TerrainMapToWorld(
        current, current_yaw, current_local);
    const TerrainMapXY delta{
        world.x - source.capture_position_world[0],
        world.y - source.capture_position_world[1]};
    return TerrainMapRotate(-source.capture_yaw_rad, delta);
}

inline TerrainMapRegistrationResult RegisterTerrainMap(
    const TerrainMapEnvelope &source, double state_stamp_s,
    const std::array<double, 3> &current_position_world, double current_yaw_rad,
    double max_age_s = kTerrainMapMaxAgeS,
    TerrainMapRegistrationPolicy policy =
        TerrainMapRegistrationPolicy::kLegacyScalarV1)
{
    TerrainMapRegistrationResult result;
    const bool known_policy =
        policy == TerrainMapRegistrationPolicy::kLegacyScalarV1 ||
        policy == TerrainMapRegistrationPolicy::kRegisteredIntervalsV2;
    if (!TerrainMapEnvelopeShapeValid(source) || source.frame_id != "base_link" ||
        !std::isfinite(state_stamp_s) || !std::isfinite(max_age_s) ||
        !(max_age_s >= 0.0) || !known_policy)
    {
        result.error = TerrainMapRegistrationError::kInvalidEnvelope;
        return result;
    }
    const double map_age = state_stamp_s - source.map_stamp_s;
    if (map_age < -kTerrainMapTimeToleranceS)
    {
        result.error = TerrainMapRegistrationError::kFutureMap;
        return result;
    }
    if (map_age > max_age_s + kTerrainMapTimeToleranceS)
    {
        result.error = TerrainMapRegistrationError::kStaleMap;
        return result;
    }
    for (std::size_t i = 0; i < source.heights_m.size(); ++i)
    {
        const bool have_height = std::isfinite(source.heights_m[i]);
        const bool have_observation =
            std::isfinite(source.observation_stamp_s[i]);
        if (have_height != have_observation ||
            (have_observation &&
             source.observation_stamp_s[i] > source.map_stamp_s +
                 kTerrainMapTimeToleranceS))
        {
            result.error = TerrainMapRegistrationError::kInvalidCellAge;
            return result;
        }
    }
    if (!std::isfinite(current_position_world[0]) ||
        !std::isfinite(current_position_world[1]) ||
        !std::isfinite(current_position_world[2]) ||
        !std::isfinite(current_yaw_rad))
    {
        result.error = TerrainMapRegistrationError::kInvalidStatePose;
        return result;
    }

    RegisteredTerrainMap registered;
    registered.sequence = source.sequence;
    registered.map_stamp_s = source.map_stamp_s;
    registered.capture_position_world = source.capture_position_world;
    registered.capture_yaw_rad = source.capture_yaw_rad;
    registered.current_position_world = current_position_world;
    registered.current_yaw_rad = current_yaw_rad;
    registered.registration_state_stamp_s = state_stamp_s;
    registered.map.stamp(source.map_stamp_s);
    registered.map.frame_id("base_link");
    registered.map.resolution(static_cast<float>(source.resolution_m));
    registered.map.width(source.width);
    registered.map.height(source.height);
    registered.map.origin() = {
        static_cast<float>(source.origin_m[0]),
        static_cast<float>(source.origin_m[1])};
    const std::size_t count = source.heights_m.size();
    registered.map.data().assign(
        count, std::numeric_limits<float>::quiet_NaN());
    registered.cell_age_s.assign(
        count, std::numeric_limits<double>::infinity());
    registered.registration_policy = policy;
    if (policy == TerrainMapRegistrationPolicy::kRegisteredIntervalsV2)
    {
        registered.cell_min_height_m.assign(
            count, kTerrainMapUnknown);
        registered.cell_max_height_m.assign(
            count, kTerrainMapUnknown);
    }

    const double eps = 1.0e-9;
    for (std::uint32_t dy = 0; dy < source.height; ++dy)
    {
        for (std::uint32_t dx = 0; dx < source.width; ++dx)
        {
            const double x0 = source.origin_m[0] +
                static_cast<double>(dx) * source.resolution_m;
            const double x1 = x0 + source.resolution_m;
            const double y0 = source.origin_m[1] +
                static_cast<double>(dy) * source.resolution_m;
            const double y1 = y0 + source.resolution_m;
            const std::array<TerrainMapXY, 4> corners{
                TerrainMapToCaptureLocal(source, current_position_world,
                                          current_yaw_rad, {x0, y0}),
                TerrainMapToCaptureLocal(source, current_position_world,
                                          current_yaw_rad, {x1, y0}),
                TerrainMapToCaptureLocal(source, current_position_world,
                                          current_yaw_rad, {x0, y1}),
                TerrainMapToCaptureLocal(source, current_position_world,
                                          current_yaw_rad, {x1, y1})};
            bool corners_finite = true;
            double min_x = corners[0].x, max_x = corners[0].x;
            double min_y = corners[0].y, max_y = corners[0].y;
            for (const TerrainMapXY &corner : corners)
            {
                if (!std::isfinite(corner.x) || !std::isfinite(corner.y))
                    corners_finite = false;
                min_x = std::min(min_x, corner.x);
                max_x = std::max(max_x, corner.x);
                min_y = std::min(min_y, corner.y);
                max_y = std::max(max_y, corner.y);
            }
            const double source_max_x = source.origin_m[0] +
                static_cast<double>(source.width) * source.resolution_m;
            const double source_max_y = source.origin_m[1] +
                static_cast<double>(source.height) * source.resolution_m;
            const bool intersects_source =
                corners_finite && std::isfinite(source_max_x) &&
                std::isfinite(source_max_y) && max_x > source.origin_m[0] &&
                min_x < source_max_x && max_y > source.origin_m[1] &&
                min_y < source_max_y;
            const double min_qx =
                (min_x - source.origin_m[0]) / source.resolution_m;
            const double max_qx =
                (max_x - source.origin_m[0]) / source.resolution_m;
            const double min_qy =
                (min_y - source.origin_m[1]) / source.resolution_m;
            const double max_qy =
                (max_y - source.origin_m[1]) / source.resolution_m;
            // Never cast a malformed or extreme floating-point index to long.
            const bool index_range_safe =
                std::isfinite(min_qx) && std::isfinite(max_qx) &&
                std::isfinite(min_qy) && std::isfinite(max_qy) &&
                std::abs(min_qx) < static_cast<double>(std::numeric_limits<long>::max() / 2) && std::abs(max_qx) < static_cast<double>(std::numeric_limits<long>::max() / 2) &&
                std::abs(min_qy) < static_cast<double>(std::numeric_limits<long>::max() / 2) && std::abs(max_qy) < static_cast<double>(std::numeric_limits<long>::max() / 2);
            const long min_ix = index_range_safe
                ? static_cast<long>(std::floor(min_qx + eps)) : 0L;
            const long max_ix = index_range_safe
                ? static_cast<long>(std::ceil(max_qx - eps)) - 1L : -1L;
            const long min_iy = index_range_safe
                ? static_cast<long>(std::floor(min_qy + eps)) : 0L;
            const long max_iy = index_range_safe
                ? static_cast<long>(std::ceil(max_qy - eps)) - 1L : -1L;
            bool known = intersects_source && index_range_safe &&
                min_ix <= max_ix && min_iy <= max_iy;
            double reference_height = kTerrainMapUnknown;
            double min_height = kTerrainMapUnknown;
            double max_height = kTerrainMapUnknown;
            double worst_age = 0.0;
            if (known)
            {
                for (long sy = min_iy; sy <= max_iy && known; ++sy)
                {
                    for (long sx = min_ix; sx <= max_ix; ++sx)
                    {
                        if (sx < 0 || sy < 0 ||
                            sx >= static_cast<long>(source.width) ||
                            sy >= static_cast<long>(source.height))
                        { known = false; break; }
                        const std::size_t source_index =
                            static_cast<std::size_t>(sy) * source.width +
                            static_cast<std::size_t>(sx);
                        const double height = source.heights_m[source_index];
                        const double observation =
                            source.observation_stamp_s[source_index];
                        const double age = state_stamp_s - observation;
                        if (!std::isfinite(height) || !std::isfinite(observation) ||
                            observation > source.map_stamp_s +
                                kTerrainMapTimeToleranceS ||
                            age < -kTerrainMapTimeToleranceS ||
                            age > max_age_s + kTerrainMapTimeToleranceS)
                        { known = false; break; }
                        if (!std::isfinite(reference_height))
                            reference_height = height;
                        if (!std::isfinite(min_height))
                        {
                            min_height = height;
                            max_height = height;
                        }
                        else
                        {
                            min_height = std::min(min_height, height);
                            max_height = std::max(max_height, height);
                        }
                        if (policy == TerrainMapRegistrationPolicy::kLegacyScalarV1 &&
                            std::abs(height - reference_height) >
                                kTerrainMapHeightToleranceM)
                        { known = false; break; }
                        worst_age = std::max(worst_age, std::max(0.0, age));
                    }
                }
            }
            const std::size_t destination_index =
                static_cast<std::size_t>(dy) * source.width + dx;
            if (known && std::isfinite(reference_height) &&
                std::isfinite(min_height) && std::isfinite(max_height))
            {
                const double z_delta = source.capture_position_world[2] -
                    current_position_world[2];
                const double registered_reference = reference_height + z_delta;
                const double registered_min = min_height + z_delta;
                const double registered_max = max_height + z_delta;
                const bool valid_bounds = std::isfinite(registered_reference) &&
                    std::isfinite(registered_min) && std::isfinite(registered_max) &&
                    registered_min <= registered_max;
                if (!valid_bounds)
                    known = false;
                else
                {
                    const double output_height = policy ==
                        TerrainMapRegistrationPolicy::kRegisteredIntervalsV2
                        ? registered_max : registered_reference;
                    const float output_height_f =
                        static_cast<float>(output_height);
                    if (!std::isfinite(output_height_f))
                        known = false;
                    else
                    {
                        registered.map.data()[destination_index] =
                            output_height_f;
                        if (policy ==
                            TerrainMapRegistrationPolicy::kRegisteredIntervalsV2)
                        {
                            registered.cell_min_height_m[destination_index] =
                                registered_min;
                            registered.cell_max_height_m[destination_index] =
                                registered_max;
                        }
                        registered.cell_age_s[destination_index] = worst_age;
                    }
                }
            }
        }
    }
    registered.registered = true;
    result.map = std::move(registered);
    return result;
}

} // namespace go2_terrain
