#include "terrain/terrain_map_envelope.h"
#include "terrain/terrain_map_transport.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

using namespace go2_terrain;

namespace
{

void Require(bool condition, const char *expression, int line)
{
    if (!condition)
    {
        std::cerr << "FAIL line=" << line << " expression=" << expression
                  << std::endl;
        std::exit(1);
    }
}

#define REQUIRE(expression) Require((expression), #expression, __LINE__)

TerrainMapEnvelope MakeEnvelope(std::uint64_t sequence, bool dense)
{
    TerrainMapEnvelope envelope;
    envelope.sequence = sequence;
    envelope.map_stamp_s = 42.0;
    envelope.frame_id = "base_link";
    envelope.resolution_m = 0.05;
    envelope.width = 32;
    envelope.height = 10;
    envelope.origin_m = {-0.8, -0.25};
    envelope.capture_position_world = {1.2, -0.4, 0.42};
    envelope.capture_yaw_rad = 0.15;

    const std::size_t count =
        static_cast<std::size_t>(envelope.width) * envelope.height;
    envelope.heights_m.assign(count, kTerrainMapUnknown);
    envelope.observation_stamp_s.assign(count, kTerrainMapUnknown);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (!dense && (i % 19U) == 0U)
            continue;
        envelope.heights_m[i] = 0.021 + 0.0001 * static_cast<double>(i % 11U);
        envelope.observation_stamp_s[i] =
            envelope.map_stamp_s - 0.01 - 0.00001 * static_cast<double>(i);
    }
    return envelope;
}

std::string EncodeEnvelope(std::uint64_t sequence, bool dense)
{
    std::string wire;
    const TerrainMapEnvelope envelope = MakeEnvelope(sequence, dense);
    REQUIRE(SerializeTerrainMapEnvelope(envelope, wire));
    const TerrainMapCodecResult decoded =
        DeserializeTerrainMapEnvelope(wire);
    REQUIRE(decoded.ok());
    REQUIRE(decoded.envelope.sequence == sequence);
    return wire;
}

void PushAll(TerrainMapChunkReassembler &reassembler,
             const std::vector<std::string> &packets,
             std::uint64_t now_ms, std::string &complete)
{
    for (const std::string &packet : packets)
    {
        const TerrainMapChunkError error =
            reassembler.PushPacket(packet, now_ms++, complete);
        REQUIRE(error == TerrainMapChunkError::kNone ||
                error == TerrainMapChunkError::kDuplicate ||
                error == TerrainMapChunkError::kComplete);
    }
}

void TestFNVVectors()
{
    REQUIRE(TerrainMapChunkFNV64("") == 0xcbf29ce484222325ULL);
    REQUIRE(TerrainMapChunkFNV64("hello") == 0xa430d84680aabd0bULL);
}

void TestEncodeDecodeAndOrder()
{
    const std::string wire = EncodeEnvelope(17, false);
    std::vector<std::string> packets;
    REQUIRE(EncodeTerrainMapWireChunks(wire, 17, packets));
    REQUIRE(packets.size() >= 2U);

    for (const std::string &packet : packets)
    {
        REQUIRE(packet.size() <= kTerrainMapChunkMaxPacketBytes);
        TerrainMapChunkHeader header;
        TerrainMapChunkError error = TerrainMapChunkError::kNone;
        REQUIRE(ParseTerrainMapChunk(packet, header, error));
        REQUIRE(error == TerrainMapChunkError::kNone);
        REQUIRE(header.sequence == 17);
        REQUIRE(header.chunk_count == packets.size());
    }

    TerrainMapChunkReassembler reassembler;
    std::string complete;
    TerrainMapChunkError last = TerrainMapChunkError::kNone;
    std::uint64_t now = 0;
    for (std::size_t i = packets.size(); i-- > 0U;)
    {
        last = reassembler.PushPacket(packets[i], now++, complete);
        if (i != 0U)
            REQUIRE(last == TerrainMapChunkError::kNone);
    }
    REQUIRE(last == TerrainMapChunkError::kComplete);
    REQUIRE(complete == wire);
    REQUIRE(reassembler.last_completed_sequence() == 17U);
    const TerrainMapChunkError stale =
        reassembler.PushPacket(packets[0], now, complete);
    REQUIRE(stale == TerrainMapChunkError::kStaleSequence);
}

void TestInterleavedAndDuplicate()
{
    const std::string wire_a = EncodeEnvelope(101, true);
    const std::string wire_b = EncodeEnvelope(102, true);
    std::vector<std::string> a;
    std::vector<std::string> b;
    REQUIRE(EncodeTerrainMapWireChunks(wire_a, 101, a));
    REQUIRE(EncodeTerrainMapWireChunks(wire_b, 102, b));
    REQUIRE(a.size() == b.size());

    TerrainMapChunkReassembler reassembler;
    std::string complete;
    std::uint64_t completed_sequence = 0;
    std::uint64_t now = 10;
    REQUIRE(reassembler.PushPacket(a[0], now++, complete) ==
            TerrainMapChunkError::kNone);
    REQUIRE(reassembler.PushPacket(a[0], now++, complete) ==
            TerrainMapChunkError::kDuplicate);
    REQUIRE(reassembler.PushPacket(b.back(), now++, complete) ==
            TerrainMapChunkError::kNone);

    for (std::size_t i = 1; i < a.size(); ++i)
    {
        const TerrainMapChunkError error =
            reassembler.PushPacket(a[i], now++, complete, &completed_sequence);
        if (i + 1U == a.size())
        {
            REQUIRE(error == TerrainMapChunkError::kComplete);
            REQUIRE(completed_sequence == 101U);
            REQUIRE(complete == wire_a);
        }
        else
            REQUIRE(error == TerrainMapChunkError::kNone);
    }

    for (std::size_t i = 0; i + 1U < b.size(); ++i)
    {
        const TerrainMapChunkError error =
            reassembler.PushPacket(b[i], now++, complete, &completed_sequence);
        if (i + 2U == b.size())
        {
            REQUIRE(error == TerrainMapChunkError::kComplete);
            REQUIRE(completed_sequence == 102U);
            REQUIRE(complete == wire_b);
        }
        else
            REQUIRE(error == TerrainMapChunkError::kNone);
    }
    REQUIRE(reassembler.in_flight() == 0U);
}

void TestConflictAndChecksum()
{
    const std::string wire = EncodeEnvelope(201, true);
    std::vector<std::string> packets;
    REQUIRE(EncodeTerrainMapWireChunks(wire, 201, packets));
    REQUIRE(packets.size() >= 2U);

    TerrainMapChunkReassembler duplicate_reassembler;
    std::string complete;
    REQUIRE(duplicate_reassembler.PushPacket(
                packets[0], 0, complete) == TerrainMapChunkError::kNone);
    std::string conflict = packets[0];
    const std::size_t payload = conflict.find("payload=\n");
    REQUIRE(payload != std::string::npos);
    REQUIRE(payload + 9U < conflict.size());
    conflict[payload + 9U] ^= 1;
    REQUIRE(duplicate_reassembler.PushPacket(
                conflict, 1, complete) ==
            TerrainMapChunkError::kDuplicateConflict);
    REQUIRE(duplicate_reassembler.PushPacket(
                packets[1], 2, complete) ==
            TerrainMapChunkError::kPoisonedSequence);

    TerrainMapChunkReassembler metadata_reassembler;
    REQUIRE(metadata_reassembler.PushPacket(
                packets[0], 0, complete) == TerrainMapChunkError::kNone);
    std::string metadata_conflict = packets[1];
    const std::size_t checksum = metadata_conflict.find("checksum=");
    REQUIRE(checksum != std::string::npos);
    metadata_conflict[checksum + 9U] =
        metadata_conflict[checksum + 9U] == '0' ? '1' : '0';
    REQUIRE(metadata_reassembler.PushPacket(
                metadata_conflict, 1, complete) ==
            TerrainMapChunkError::kMetadataConflict);

    TerrainMapChunkReassembler checksum_reassembler;
    std::vector<std::string> corrupted = packets;
    const std::size_t last_payload = corrupted.back().find("payload=\n");
    REQUIRE(last_payload != std::string::npos);
    REQUIRE(last_payload + 9U < corrupted.back().size());
    corrupted.back()[last_payload + 9U] ^= 1;
    TerrainMapChunkError final_error = TerrainMapChunkError::kNone;
    for (const std::string &packet : corrupted)
        final_error = checksum_reassembler.PushPacket(packet, 0, complete);
    REQUIRE(final_error == TerrainMapChunkError::kChecksumMismatch);
    REQUIRE(complete.empty());
}

void TestExpiryCapacityAndTime()
{
    const std::string wire = EncodeEnvelope(301, true);
    std::vector<std::string> a;
    std::vector<std::string> b;
    std::vector<std::string> c;
    REQUIRE(EncodeTerrainMapWireChunks(wire, 301, a));
    REQUIRE(EncodeTerrainMapWireChunks(wire, 302, b));
    REQUIRE(EncodeTerrainMapWireChunks(wire, 303, c));

    TerrainMapChunkReassembler reassembler;
    std::string complete;
    REQUIRE(reassembler.PushPacket(a[0], 0, complete) ==
            TerrainMapChunkError::kNone);
    REQUIRE(reassembler.PushPacket(b[0], 0, complete) ==
            TerrainMapChunkError::kNone);
    // A newer sequence deterministically evicts the oldest incomplete slot.
    REQUIRE(reassembler.PushPacket(c[0], 1, complete) ==
            TerrainMapChunkError::kNone);
    REQUIRE(reassembler.in_flight() == 2U);
    for (std::size_t i = 1; i < c.size(); ++i)
    {
        const TerrainMapChunkError error =
            reassembler.PushPacket(c[i], 2 + i, complete);
        if (i + 1U == c.size())
            REQUIRE(error == TerrainMapChunkError::kComplete);
        else
            REQUIRE(error == TerrainMapChunkError::kNone);
    }
    REQUIRE(complete == wire);
    REQUIRE(reassembler.last_completed_sequence() == 303U);
    REQUIRE(reassembler.PushPacket(
                a[0], 100, complete) == TerrainMapChunkError::kStaleSequence);

    TerrainMapChunkReassembler expiry;
    REQUIRE(expiry.PushPacket(a[0], 100, complete) ==
            TerrainMapChunkError::kNone);
    REQUIRE(expiry.PurgeExpired(
                100 + kTerrainMapChunkAssemblyTimeoutMs) == 1U);
    REQUIRE(expiry.in_flight() == 0U);
    REQUIRE(expiry.PushPacket(
                a[1], 100 + kTerrainMapChunkAssemblyTimeoutMs, complete) ==
            TerrainMapChunkError::kStaleSequence);

    TerrainMapChunkReassembler regression;
    REQUIRE(regression.PushPacket(a[0], 500, complete) ==
            TerrainMapChunkError::kNone);
    REQUIRE(regression.PushPacket(
                a[1], 499, complete) == TerrainMapChunkError::kTimeRegression);
    REQUIRE(regression.in_flight() == 0U);
    REQUIRE(regression.PushPacket(
                a[0], 500, complete) == TerrainMapChunkError::kStaleSequence);
}

void TestMalformedAndBounds()
{
    const std::string wire = EncodeEnvelope(401, true);
    std::vector<std::string> packets;
    REQUIRE(EncodeTerrainMapWireChunks(wire, 401, packets));

    TerrainMapChunkHeader header;
    TerrainMapChunkError error = TerrainMapChunkError::kNone;
    std::string bad_magic = packets[0];
    bad_magic[0] = 'X';
    REQUIRE(!ParseTerrainMapChunk(bad_magic, header, error));
    REQUIRE(error == TerrainMapChunkError::kBadMagic);

    std::string oversized = packets[0];
    oversized.append(kTerrainMapChunkMaxPacketBytes, 'x');
    REQUIRE(!ParseTerrainMapChunk(oversized, header, error));
    REQUIRE(error == TerrainMapChunkError::kTooLarge);

    std::vector<std::string> max_packets;
    REQUIRE(EncodeTerrainMapWireChunks(
                std::string(kTerrainMapChunkMaxWireBytes, 'x'), 402,
                max_packets));
    REQUIRE(max_packets.size() <= kTerrainMapChunkMaxCount);
    for (const std::string &packet : max_packets)
        REQUIRE(packet.size() <= kTerrainMapChunkMaxPacketBytes);
    REQUIRE(!EncodeTerrainMapWireChunks(
                std::string(kTerrainMapChunkMaxWireBytes + 1U, 'x'), 403,
                max_packets));
}

std::string MakeBenchmarkWire()
{
    // Keep this fixture a valid envelope while matching the 13,472-byte DDS
    // sample that established the native String_ fragmentation boundary.
    TerrainMapEnvelope envelope = MakeEnvelope(9000, true);
    envelope.frame_id.append(14U, 'F');
    envelope.width = 29;
    envelope.height = 12;
    const std::size_t count =
        static_cast<std::size_t>(envelope.width) * envelope.height;
    envelope.heights_m.resize(count);
    envelope.observation_stamp_s.resize(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        envelope.heights_m[i] =
            -123456789012345.0 + static_cast<double>(i) * 0.125;
        envelope.observation_stamp_s[i] =
            987654321012345.0 + static_cast<double>(i) * 0.125;
    }
    envelope.heights_m[0] = kTerrainMapUnknown;
    envelope.observation_stamp_s[0] = kTerrainMapUnknown;
    envelope.heights_m[1] = kTerrainMapUnknown;
    envelope.observation_stamp_s[1] = kTerrainMapUnknown;
    std::string wire;
    REQUIRE(SerializeTerrainMapEnvelope(envelope, wire));
    REQUIRE(wire.size() == 13472U);
    REQUIRE(DeserializeTerrainMapEnvelope(wire).ok());
    return wire;
}
void RunBenchmark(const std::string &wire)
{
    constexpr std::size_t warmup = 20U;
    constexpr std::size_t iterations = 200U;
    std::vector<long long> elapsed_us;
    elapsed_us.reserve(iterations);
    std::size_t total_bytes = 0;
    std::size_t total_packets = 0;
    volatile std::size_t optimizer_sink = 0;

    for (std::size_t iteration = 0; iteration < warmup + iterations; ++iteration)
    {
        const std::uint64_t sequence = 9001U + iteration;
        const auto begin = std::chrono::steady_clock::now();
        std::vector<std::string> packets;
        REQUIRE(EncodeTerrainMapWireChunks(wire, sequence, packets));
        TerrainMapChunkReassembler reassembler;
        std::string complete;
        std::uint64_t completed_sequence = 0;
        std::uint64_t now = 100000U + iteration * 1000U;
        TerrainMapChunkError final_error = TerrainMapChunkError::kNone;
        for (const std::string &packet : packets)
        {
            final_error = reassembler.PushPacket(
                packet, now++, complete, &completed_sequence);
            optimizer_sink += packet.size();
        }
        const auto end = std::chrono::steady_clock::now();
        REQUIRE(final_error == TerrainMapChunkError::kComplete);
        REQUIRE(completed_sequence == sequence);
        REQUIRE(complete == wire);
        if (iteration >= warmup)
        {
            elapsed_us.push_back(std::chrono::duration_cast<
                std::chrono::microseconds>(end - begin).count());
            total_bytes += wire.size();
            total_packets += packets.size();
        }
    }

    std::sort(elapsed_us.begin(), elapsed_us.end());
    const auto percentile = [&](std::size_t numerator) {
        const std::size_t index = std::min(
            elapsed_us.size() - 1U,
            (elapsed_us.size() * numerator) / 100U);
        return elapsed_us[index];
    };
    optimizer_sink += total_bytes + total_packets;
#ifdef NDEBUG
    const char *build = "true";
#else
    const char *build = "false";
#endif
    std::cout << "terrain_map_transport ndebug=" << build
              << " wire_bytes=" << wire.size()
              << " packets=" << (wire.size() + kTerrainMapChunkPayloadBytes - 1U) /
                    kTerrainMapChunkPayloadBytes
              << " packet_max=1000"
              << " p50_us=" << percentile(50U)
              << " p95_us=" << percentile(95U)
              << " max_us=" << elapsed_us.back()
              << " total_bytes=" << total_bytes
              << " known_sink=" << optimizer_sink << std::endl;
}

} // namespace

int main()
{
    TestFNVVectors();
    TestEncodeDecodeAndOrder();
    TestInterleavedAndDuplicate();
    TestConflictAndChecksum();
    TestExpiryCapacityAndTime();
    TestMalformedAndBounds();
    const std::string benchmark_wire = MakeBenchmarkWire();
    RunBenchmark(benchmark_wire);
    std::cout << "PASS terrain_map_transport" << std::endl;
    return 0;
}
