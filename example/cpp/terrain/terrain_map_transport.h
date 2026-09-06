#pragma once

// Bounded application-layer transport for the versioned terrain-map envelope.
// Every DDS String sample remains below 1000 bytes. Only a complete,
// checksum-verified original wire is exposed to the envelope codec.

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace go2_terrain
{

constexpr std::size_t kTerrainMapChunkMaxPacketBytes = 1000U;
constexpr std::size_t kTerrainMapChunkPayloadBytes = 800U;
constexpr std::size_t kTerrainMapChunkMaxWireBytes = 256U * 1024U;
constexpr std::uint32_t kTerrainMapChunkMaxCount = 512U;
constexpr std::size_t kTerrainMapChunkMaxInFlight = 2U;
constexpr std::uint64_t kTerrainMapChunkAssemblyTimeoutMs = 2000U;
constexpr std::uint64_t kTerrainMapChunkFNVOffset = 14695981039346656037ULL;
constexpr std::uint64_t kTerrainMapChunkFNVPrime = 1099511628211ULL;
constexpr std::string_view kTerrainMapChunkMagic =
    "GO2_TERRAIN_MAP_CHUNK_V1";

inline std::uint64_t TerrainMapChunkFNV64(std::string_view value)
{
    std::uint64_t hash = kTerrainMapChunkFNVOffset;
    for (const unsigned char byte : value)
    {
        hash ^= byte;
        hash *= kTerrainMapChunkFNVPrime;
    }
    return hash;
}

inline bool TerrainMapChunkAppendUint(std::string &out, std::uint64_t value)
{
    char buffer[32]{};
    const auto converted = std::to_chars(
        buffer, buffer + sizeof(buffer), value, 10);
    if (converted.ec != std::errc{})
        return false;
    out.append(buffer, converted.ptr);
    return true;
}

inline bool TerrainMapChunkParseUint(std::string_view token,
                                     std::uint64_t &value)
{
    if (token.empty())
        return false;
    for (const char c : token)
    {
        if (c < '0' || c > '9')
            return false;
    }
    const auto parsed = std::from_chars(
        token.data(), token.data() + token.size(), value, 10);
    return parsed.ec == std::errc{} && parsed.ptr == token.data() + token.size();
}

inline bool TerrainMapChunkAppendHex(std::string &out, std::uint64_t value)
{
    char buffer[32]{};
    const auto converted = std::to_chars(
        buffer, buffer + sizeof(buffer), value, 16);
    if (converted.ec != std::errc{})
        return false;
    const std::size_t digits = static_cast<std::size_t>(
        converted.ptr - buffer);
    if (digits > 16U)
        return false;
    out.append(16U - digits, '0');
    out.append(buffer, converted.ptr);
    return true;
}

inline bool TerrainMapChunkParseHex(std::string_view token,
                                    std::uint64_t &value)
{
    if (token.size() != 16U)
        return false;
    value = 0;
    for (const char c : token)
    {
        std::uint8_t nibble = 0;
        if (c >= '0' && c <= '9')
            nibble = static_cast<std::uint8_t>(c - '0');
        else if (c >= 'a' && c <= 'f')
            nibble = static_cast<std::uint8_t>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F')
            nibble = static_cast<std::uint8_t>(c - 'A' + 10);
        else
            return false;
        value = (value << 4U) | nibble;
    }
    return true;
}

enum class TerrainMapChunkError : std::uint8_t
{
    kNone = 0,
    kComplete,
    kDuplicate,
    kEmpty,
    kTooLarge,
    kBadMagic,
    kBadHeader,
    kBadNumber,
    kBadShape,
    kPayloadSize,
    kChecksumMismatch,
    kDuplicateConflict,
    kMetadataConflict,
    kStaleSequence,
    kTooManyInFlight,
    kPoisonedSequence,
    kTimeRegression,
};

struct TerrainMapChunkHeader
{
    std::uint64_t sequence = 0;
    std::uint32_t chunk_index = 0;
    std::uint32_t chunk_count = 0;
    std::uint64_t total_bytes = 0;
    std::uint64_t checksum = 0;
    std::string_view payload{};
};

inline bool TerrainMapChunkTakeLine(std::string_view packet, std::size_t &cursor,
                                    std::string_view &line)
{
    if (cursor >= packet.size())
        return false;
    const std::size_t end = packet.find('\n', cursor);
    if (end == std::string_view::npos || end == cursor)
        return false;
    line = packet.substr(cursor, end - cursor);
    cursor = end + 1U;
    return line.find('\r') == std::string_view::npos;
}

inline bool TerrainMapChunkField(std::string_view line, std::string_view key,
                                 std::string_view &value)
{
    if (line.size() <= key.size() || line.substr(0, key.size()) != key ||
        line[key.size()] != '=')
        return false;
    value = line.substr(key.size() + 1U);
    if (value.empty())
        return false;
    for (const char c : value)
    {
        if (c == '=' || c == '\r' || c == '\n' || c == ' ' || c == '\t')
            return false;
    }
    return true;
}

inline bool ParseTerrainMapChunk(std::string_view packet,
                                 TerrainMapChunkHeader &header,
                                 TerrainMapChunkError &error)
{
    error = TerrainMapChunkError::kNone;
    if (packet.empty())
    {
        error = TerrainMapChunkError::kEmpty;
        return false;
    }
    if (packet.size() > kTerrainMapChunkMaxPacketBytes)
    {
        error = TerrainMapChunkError::kTooLarge;
        return false;
    }

    std::size_t cursor = 0;
    std::string_view line;
    if (!TerrainMapChunkTakeLine(packet, cursor, line))
    {
        error = TerrainMapChunkError::kBadHeader;
        return false;
    }
    if (line != kTerrainMapChunkMagic)
    {
        error = TerrainMapChunkError::kBadMagic;
        return false;
    }

    std::string_view value;
    if (!TerrainMapChunkTakeLine(packet, cursor, line) ||
        !TerrainMapChunkField(line, "seq", value) ||
        !TerrainMapChunkParseUint(value, header.sequence))
    {
        error = TerrainMapChunkError::kBadNumber;
        return false;
    }
    std::uint64_t wide = 0;
    if (!TerrainMapChunkTakeLine(packet, cursor, line) ||
        !TerrainMapChunkField(line, "chunk", value) ||
        !TerrainMapChunkParseUint(value, wide) ||
        wide > std::numeric_limits<std::uint32_t>::max())
    {
        error = TerrainMapChunkError::kBadNumber;
        return false;
    }
    header.chunk_index = static_cast<std::uint32_t>(wide);
    if (!TerrainMapChunkTakeLine(packet, cursor, line) ||
        !TerrainMapChunkField(line, "count", value) ||
        !TerrainMapChunkParseUint(value, wide) ||
        wide > std::numeric_limits<std::uint32_t>::max())
    {
        error = TerrainMapChunkError::kBadNumber;
        return false;
    }
    header.chunk_count = static_cast<std::uint32_t>(wide);
    if (!TerrainMapChunkTakeLine(packet, cursor, line) ||
        !TerrainMapChunkField(line, "total", value) ||
        !TerrainMapChunkParseUint(value, header.total_bytes))
    {
        error = TerrainMapChunkError::kBadNumber;
        return false;
    }
    if (!TerrainMapChunkTakeLine(packet, cursor, line) ||
        !TerrainMapChunkField(line, "checksum", value) ||
        !TerrainMapChunkParseHex(value, header.checksum))
    {
        error = TerrainMapChunkError::kBadNumber;
        return false;
    }
    if (!TerrainMapChunkTakeLine(packet, cursor, line) || line != "payload=")
    {
        error = TerrainMapChunkError::kBadHeader;
        return false;
    }
    if (header.sequence == 0 || header.chunk_count == 0 ||
        header.chunk_count > kTerrainMapChunkMaxCount ||
        header.chunk_index >= header.chunk_count || header.total_bytes == 0 ||
        header.total_bytes > kTerrainMapChunkMaxWireBytes)
    {
        error = TerrainMapChunkError::kBadShape;
        return false;
    }
    const std::uint64_t expected_count =
        (header.total_bytes + kTerrainMapChunkPayloadBytes - 1U) /
        kTerrainMapChunkPayloadBytes;
    if (expected_count != header.chunk_count)
    {
        error = TerrainMapChunkError::kBadShape;
        return false;
    }
    const std::uint64_t offset =
        static_cast<std::uint64_t>(header.chunk_index) *
        kTerrainMapChunkPayloadBytes;
    const std::uint64_t remaining = header.total_bytes - offset;
    const std::size_t expected_payload = static_cast<std::size_t>(
        remaining > kTerrainMapChunkPayloadBytes
            ? kTerrainMapChunkPayloadBytes : remaining);
    header.payload = packet.substr(cursor);
    if (header.payload.size() != expected_payload)
    {
        error = TerrainMapChunkError::kPayloadSize;
        return false;
    }
    return true;
}

inline bool EncodeTerrainMapWireChunks(std::string_view wire,
                                       std::uint64_t sequence,
                                       std::vector<std::string> &packets)
{
    packets.clear();
    if (wire.empty() || wire.size() > kTerrainMapChunkMaxWireBytes ||
        sequence == 0)
        return false;
    const std::uint64_t count =
        (wire.size() + kTerrainMapChunkPayloadBytes - 1U) /
        kTerrainMapChunkPayloadBytes;
    if (count == 0 || count > kTerrainMapChunkMaxCount)
        return false;

    const std::uint64_t checksum = TerrainMapChunkFNV64(wire);
    packets.reserve(static_cast<std::size_t>(count));
    for (std::uint64_t index = 0; index < count; ++index)
    {
        const std::size_t offset = static_cast<std::size_t>(
            index * kTerrainMapChunkPayloadBytes);
        const std::size_t length = std::min(
            kTerrainMapChunkPayloadBytes, wire.size() - offset);
        std::string packet;
        packet.reserve(180U + length);
        packet.append(kTerrainMapChunkMagic);
        packet.push_back('\n');
        packet.append("seq=");
        if (!TerrainMapChunkAppendUint(packet, sequence)) return false;
        packet.push_back('\n');
        packet.append("chunk=");
        if (!TerrainMapChunkAppendUint(packet, index)) return false;
        packet.push_back('\n');
        packet.append("count=");
        if (!TerrainMapChunkAppendUint(packet, count)) return false;
        packet.push_back('\n');
        packet.append("total=");
        if (!TerrainMapChunkAppendUint(packet, wire.size())) return false;
        packet.push_back('\n');
        packet.append("checksum=");
        if (!TerrainMapChunkAppendHex(packet, checksum)) return false;
        packet.push_back('\n');
        packet.append("payload=\n");
        packet.append(wire.data() + offset, length);
        if (packet.size() > kTerrainMapChunkMaxPacketBytes)
        {
            packets.clear();
            return false;
        }
        packets.push_back(std::move(packet));
    }
    return packets.size() == count;
}

class TerrainMapChunkReassembler
{
public:
    TerrainMapChunkReassembler() = default;

    TerrainMapChunkError PushPacket(std::string_view packet,
                                    std::uint64_t now_ms,
                                    std::string &complete_wire,
                                    std::uint64_t *complete_sequence = nullptr)
    {
        complete_wire.clear();
        if (have_time_ && now_ms < last_now_ms_)
        {
            for (Slot &slot : slots_)
                Reset(slot);
            last_now_ms_ = now_ms;
            return TerrainMapChunkError::kTimeRegression;
        }
        have_time_ = true;
        last_now_ms_ = now_ms;
        if (complete_sequence != nullptr)
            *complete_sequence = 0;
        PurgeExpired(now_ms);

        TerrainMapChunkHeader header;
        TerrainMapChunkError error = TerrainMapChunkError::kNone;
        if (!ParseTerrainMapChunk(packet, header, error))
            return error;
        if (header.sequence <= last_completed_sequence_)
            return TerrainMapChunkError::kStaleSequence;

        Slot *slot = Find(header.sequence);
        if (slot == nullptr)
        {
            if (header.sequence <= highest_seen_sequence_)
                return TerrainMapChunkError::kStaleSequence;
            slot = Allocate(header, now_ms);
            if (slot == nullptr)
            {
                slot = EvictOldest();
                if (slot == nullptr)
                    return TerrainMapChunkError::kTooManyInFlight;
                slot = Allocate(header, now_ms, slot);
            }
            highest_seen_sequence_ = header.sequence;
        }
        if (slot->poisoned)
            return TerrainMapChunkError::kPoisonedSequence;
        if (slot->chunk_count != header.chunk_count ||
            slot->total_bytes != header.total_bytes ||
            slot->checksum != header.checksum)
        {
            slot->poisoned = true;
            return TerrainMapChunkError::kMetadataConflict;
        }

        const std::size_t index = header.chunk_index;
        if (BitIsSet(*slot, index))
        {
            const std::size_t offset = index * kTerrainMapChunkPayloadBytes;
            if (std::memcmp(slot->wire.data() + offset,
                            header.payload.data(), header.payload.size()) == 0)
                return TerrainMapChunkError::kDuplicate;
            slot->poisoned = true;
            return TerrainMapChunkError::kDuplicateConflict;
        }

        const std::size_t offset = index * kTerrainMapChunkPayloadBytes;
        std::memcpy(slot->wire.data() + offset,
                    header.payload.data(), header.payload.size());
        SetBit(*slot, index);
        ++slot->received_count;
        if (slot->received_count != slot->chunk_count)
            return TerrainMapChunkError::kNone;

        if (TerrainMapChunkFNV64(slot->wire) != slot->checksum)
        {
            slot->poisoned = true;
            return TerrainMapChunkError::kChecksumMismatch;
        }
        if (header.sequence <= last_completed_sequence_)
        {
            Reset(*slot);
            return TerrainMapChunkError::kStaleSequence;
        }
        complete_wire = std::move(slot->wire);
        if (complete_sequence != nullptr)
            *complete_sequence = header.sequence;
        last_completed_sequence_ = header.sequence;
        Reset(*slot);
        for (Slot &other : slots_)
        {
            if (other.active && other.sequence <= last_completed_sequence_)
                Reset(other);
        }
        return TerrainMapChunkError::kComplete;
    }

    std::size_t PurgeExpired(std::uint64_t now_ms)
    {
        std::size_t purged = 0;
        for (Slot &slot : slots_)
        {
            if (slot.active && now_ms >= slot.started_ms &&
                now_ms - slot.started_ms >= kTerrainMapChunkAssemblyTimeoutMs)
            {
                Reset(slot);
                ++purged;
            }
        }
        return purged;
    }

    std::uint64_t last_completed_sequence() const
    {
        return last_completed_sequence_;
    }

    std::uint64_t highest_seen_sequence() const
    {
        return highest_seen_sequence_;
    }

    std::size_t in_flight() const
    {
        std::size_t count = 0;
        for (const Slot &slot : slots_)
            count += slot.active ? 1U : 0U;
        return count;
    }

private:
    struct Slot
    {
        bool active = false;
        bool poisoned = false;
        std::uint64_t sequence = 0;
        std::uint32_t chunk_count = 0;
        std::uint32_t received_count = 0;
        std::uint64_t total_bytes = 0;
        std::uint64_t checksum = 0;
        std::uint64_t started_ms = 0;
        std::array<std::uint64_t, 8> received_bits{};
        std::string wire;
    };

    static bool BitIsSet(const Slot &slot, std::size_t index)
    {
        return (slot.received_bits[index / 64U] &
                (std::uint64_t{1} << (index % 64U))) != 0;
    }

    static void SetBit(Slot &slot, std::size_t index)
    {
        slot.received_bits[index / 64U] |=
            (std::uint64_t{1} << (index % 64U));
    }

    static void Reset(Slot &slot)
    {
        slot = {};
    }

    Slot *Find(std::uint64_t sequence)
    {
        for (Slot &slot : slots_)
        {
            if (slot.active && slot.sequence == sequence)
                return &slot;
        }
        return nullptr;
    }

    Slot *Allocate(const TerrainMapChunkHeader &header,
                   std::uint64_t now_ms, Slot *preferred = nullptr)
    {
        Slot *free_slot = preferred;
        for (Slot &slot : slots_)
        {
            if (!slot.active)
            {
                free_slot = &slot;
                break;
            }
        }
        if (free_slot == nullptr)
            return nullptr;
        free_slot->active = true;
        free_slot->sequence = header.sequence;
        free_slot->chunk_count = header.chunk_count;
        free_slot->total_bytes = header.total_bytes;
        free_slot->checksum = header.checksum;
        free_slot->started_ms = now_ms;
        free_slot->wire.assign(static_cast<std::size_t>(header.total_bytes), '\0');
        free_slot->received_bits.fill(0);
        return free_slot;
    }

    Slot *EvictOldest()
    {
        Slot *oldest = nullptr;
        for (Slot &slot : slots_)
        {
            if (!slot.active)
                continue;
            if (oldest == nullptr || slot.started_ms < oldest->started_ms ||
                (slot.started_ms == oldest->started_ms &&
                 slot.sequence < oldest->sequence))
                oldest = &slot;
        }
        if (oldest != nullptr)
            Reset(*oldest);
        return oldest;
    }

    std::array<Slot, kTerrainMapChunkMaxInFlight> slots_{};
    bool have_time_ = false;
    std::uint64_t last_now_ms_ = 0;
    std::uint64_t highest_seen_sequence_ = 0;
    std::uint64_t last_completed_sequence_ = 0;
};

inline const char *TerrainMapChunkErrorName(TerrainMapChunkError error)
{
    switch (error)
    {
    case TerrainMapChunkError::kNone: return "accepted";
    case TerrainMapChunkError::kComplete: return "complete";
    case TerrainMapChunkError::kDuplicate: return "duplicate";
    case TerrainMapChunkError::kEmpty: return "empty";
    case TerrainMapChunkError::kTooLarge: return "too_large";
    case TerrainMapChunkError::kBadMagic: return "bad_magic";
    case TerrainMapChunkError::kBadHeader: return "bad_header";
    case TerrainMapChunkError::kBadNumber: return "bad_number";
    case TerrainMapChunkError::kBadShape: return "bad_shape";
    case TerrainMapChunkError::kPayloadSize: return "payload_size";
    case TerrainMapChunkError::kChecksumMismatch: return "checksum_mismatch";
    case TerrainMapChunkError::kDuplicateConflict: return "duplicate_conflict";
    case TerrainMapChunkError::kMetadataConflict: return "metadata_conflict";
    case TerrainMapChunkError::kStaleSequence: return "stale_sequence";
    case TerrainMapChunkError::kTooManyInFlight: return "too_many_in_flight";
    case TerrainMapChunkError::kPoisonedSequence: return "poisoned_sequence";
    case TerrainMapChunkError::kTimeRegression: return "time_regression";
    }
    return "unknown";
}

} // namespace go2_terrain