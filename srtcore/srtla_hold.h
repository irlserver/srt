/*
 * SRT - Secure, Reliable, Transport
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#ifndef INC_SRT_SRTLA_HOLD_H
#define INC_SRT_SRTLA_HOLD_H

#include "platform_sys.h"

#include "sync.h"

namespace srt
{

/// Estimates how long a gap in the sequence stream must persist before it can
/// honestly be called a loss, for connections fed by an srtla bonded link.
///
/// srtla merges several links onto one socket without resequencing, so a
/// missing sequence number is usually not lost at all: it is still in flight on
/// a slower member. The wait it deserves is the spread between the fastest and
/// slowest link.
///
/// The spread is recovered without knowing which link any packet arrived on.
/// Relative transit (arrival time minus sender timestamp) clusters by link, one
/// cluster per path delay, so the spread is the width of that distribution. The
/// sender's clock offset and its drift are common to every cluster and cancel
/// in the difference, which is why no clock synchronisation is needed.
///
/// Width is measured as the gap between the smallest and largest transit seen,
/// each smoothed across fixed time buckets rather than across raw samples: a
/// bucket's extreme is a percentile over many packets, so a single badly queued
/// packet cannot move the estimate on its own.
///
/// A connection carrying one link has no spread and yields a hold of zero,
/// which is what makes this inert for non-bonded traffic.
class SrtlaReorderHold
{
public:
    SrtlaReorderHold();

    /// Discard all measurement state.
    void reset();

    /// Fold in one sample. Call once per received *original* data packet.
    ///
    /// Retransmits must not be passed: they carry the timestamp of the first
    /// send, so their apparent transit includes the whole detect-and-resend
    /// round trip and would read as an enormous reorder.
    ///
    /// @param sender_timestamp raw 32-bit SRT header timestamp, as received
    /// @param arrival          local arrival time, monotonic
    void onOriginal(uint32_t sender_timestamp, const sync::steady_clock::time_point& arrival);

    /// Measured hold in microseconds, or 0 when no spread has been observed
    /// (single link, or too early to say).
    ///
    /// @param tsbpd_delay_ms receiver latency budget; the hold is capped at
    ///        half of it, since whatever is held back still has to be
    ///        retransmitted and delivered within what remains.
    uint32_t holdUs(int tsbpd_delay_ms) const;

private:
    // Bucket length. Long enough that a bucket spans many packets, short
    // enough that sender clock drift inside one bucket is irrelevant.
    static const int64_t BUCKET_US = 200000;
    // Weight (1/N) applied to each completed bucket.
    static const int     EWMA_DIV = 8;
    // Ceiling on the reported hold. Past this a link is better treated as
    // failed than allowed to defer every loss report behind it.
    static const int64_t CAP_US = 500000;

    bool     m_bInit;
    uint32_t m_uLastRawTs;  //< previous raw sender timestamp, for unwrapping
    int64_t  m_iTsAcc;      //< accumulated signed sender-timestamp deltas since the anchor

    sync::steady_clock::time_point m_tsAnchor;      //< arrival the accumulator is measured against
    sync::steady_clock::time_point m_tsBucketStart;

    bool    m_bBucketHasSample;
    int64_t m_iBucketMinUs;
    int64_t m_iBucketMaxUs;

    bool   m_bSpreadValid;
    double m_dRelMinUs; //< EWMA of per-bucket minima
    double m_dRelMaxUs; //< EWMA of per-bucket maxima

    sync::atomic<uint32_t> m_uHoldUs; //< published estimate, read without a lock
};

} // namespace srt

#endif
