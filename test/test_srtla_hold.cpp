#include <cstdlib>

#include "gtest/gtest.h"

#include "srtla_hold.h"
#include "sync.h"

using namespace srt;
using namespace srt::sync;

namespace
{

/// Feed `seconds` of synthetic traffic at `pps`, cycling through `nlinks` path
/// delays, and return the resulting hold.
///
/// The sender timestamp origin sits just below the 32-bit wrap so every case
/// also exercises the unwrapping.
uint32_t runLinks(const int64_t* link_delay_us, int nlinks, int pps, double seconds, int latency_ms)
{
    SrtlaReorderHold hold;
    const steady_clock::time_point base = steady_clock::now();

    const int64_t  step_us   = 1000000 / pps;
    const long     npackets  = long(seconds * pps);
    const uint32_t ts_origin = 0xFFFF0000u;

    for (long i = 0; i < npackets; ++i)
    {
        const int64_t  send_us   = i * step_us;
        const uint32_t sender_ts = ts_origin + uint32_t(send_us);
        const int64_t  arrive_us = send_us + link_delay_us[i % nlinks];
        hold.onOriginal(sender_ts, base + microseconds_from(arrive_us));
    }
    return hold.holdUs(latency_ms);
}

} // namespace

// The spread between the fastest and slowest link is recovered from the merged
// stream alone, without knowing which link any packet arrived on.
TEST(SrtlaHold, RecoversInterLinkSpread)
{
    const int64_t links[] = {5000, 45000};
    const uint32_t hold = runLinks(links, 2, 1000, 4.0, 4000);
    EXPECT_GE(hold, 36000u);
    EXPECT_LE(hold, 44000u);
}

// The estimate must not move with packet rate. A packet-count reorder window
// narrows as bitrate rises, which is precisely the failure this replaces, so a
// bitrate-dependent estimate here would reintroduce it.
TEST(SrtlaHold, IndependentOfPacketRate)
{
    const int64_t links[] = {5000, 45000};
    const uint32_t slow = runLinks(links, 2, 1000, 4.0, 4000);
    const uint32_t fast = runLinks(links, 2, 5000, 4.0, 4000);

    EXPECT_GE(fast, 36000u);
    EXPECT_LE(fast, 44000u);
    EXPECT_LT(fast > slow ? fast - slow : slow - fast, 8000u);
}

// One link has no spread, so the hold is zero and loss reporting is left
// exactly as stock SRT would do it.
TEST(SrtlaHold, SingleLinkYieldsNoHold)
{
    const int64_t links[] = {5000};
    EXPECT_LE(runLinks(links, 1, 1000, 4.0, 4000), 2000u);
}

// With more than two links it is the widest pair that has to be tolerated.
TEST(SrtlaHold, TracksWidestPairOfThreeLinks)
{
    const int64_t links[] = {10000, 55000, 100000};
    const uint32_t hold = runLinks(links, 3, 1000, 4.0, 4000);
    EXPECT_GE(hold, 84000u);
    EXPECT_LE(hold, 96000u);
}

// Whatever is held back still has to be retransmitted and delivered inside the
// latency budget, so the hold may never exceed half of it.
TEST(SrtlaHold, ClampedToHalfTheLatencyBudget)
{
    const int64_t links[] = {5000, 45000};
    EXPECT_EQ(runLinks(links, 2, 1000, 4.0, 40), 20000u);
    // Comfortably under the cap, so it passes through untouched.
    EXPECT_GE(runLinks(links, 2, 1000, 4.0, 120), 36000u);
}

// Beyond the ceiling a link is better treated as failed than allowed to defer
// every loss report behind it.
TEST(SrtlaHold, HardCapOnAbsurdSpread)
{
    const int64_t links[] = {0, 3000000};
    EXPECT_EQ(runLinks(links, 2, 1000, 4.0, 30000), 500000u);
}

// A fresh instance reports nothing until it has measured something.
TEST(SrtlaHold, ReportsNothingBeforeMeasuring)
{
    SrtlaReorderHold hold;
    EXPECT_EQ(hold.holdUs(4000), 0u);

    hold.onOriginal(12345, steady_clock::now());
    EXPECT_EQ(hold.holdUs(4000), 0u);
}
