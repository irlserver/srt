/*
 * SRT - Secure, Reliable, Transport
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#include "platform_sys.h"

#include "srtla_hold.h"

using namespace srt;
using namespace srt::sync;

srt::SrtlaReorderHold::SrtlaReorderHold()
{
    reset();
}

void srt::SrtlaReorderHold::reset()
{
    m_bInit            = false;
    m_uLastRawTs       = 0;
    m_iTsAcc           = 0;
    m_bBucketHasSample = false;
    m_iBucketMinUs     = 0;
    m_iBucketMaxUs     = 0;
    m_bSpreadValid     = false;
    m_dRelMinUs        = 0.0;
    m_dRelMaxUs        = 0.0;
    m_uHoldUs.store(0);
}

void srt::SrtlaReorderHold::onOriginal(uint32_t sender_timestamp, const steady_clock::time_point& arrival)
{
    if (!m_bInit)
    {
        m_bInit         = true;
        m_uLastRawTs    = sender_timestamp;
        m_iTsAcc        = 0;
        m_tsAnchor      = arrival;
        m_tsBucketStart = arrival;
        return;
    }

    // Accumulating the sender timestamp as signed 32-bit deltas is both
    // wrap-safe (the field rolls over about every 71 minutes) and reorder-safe
    // (a timestamp preceding its predecessor contributes a negative delta
    // rather than an apparent forward jump of some 4295 seconds).
    m_iTsAcc += int32_t(sender_timestamp - m_uLastRawTs);
    m_uLastRawTs = sender_timestamp;

    const int64_t rel = count_microseconds(arrival - m_tsAnchor) - m_iTsAcc;

    if (!m_bBucketHasSample)
    {
        m_bBucketHasSample = true;
        m_iBucketMinUs     = rel;
        m_iBucketMaxUs     = rel;
    }
    else
    {
        if (rel < m_iBucketMinUs)
            m_iBucketMinUs = rel;
        if (rel > m_iBucketMaxUs)
            m_iBucketMaxUs = rel;
    }

    if (count_microseconds(arrival - m_tsBucketStart) < BUCKET_US)
        return;

    if (!m_bSpreadValid)
    {
        m_dRelMinUs    = double(m_iBucketMinUs);
        m_dRelMaxUs    = double(m_iBucketMaxUs);
        m_bSpreadValid = true;
    }
    else
    {
        m_dRelMinUs += (double(m_iBucketMinUs) - m_dRelMinUs) / EWMA_DIV;
        m_dRelMaxUs += (double(m_iBucketMaxUs) - m_dRelMaxUs) / EWMA_DIV;
    }

    m_tsBucketStart    = arrival;
    m_bBucketHasSample = false;

    double spread = m_dRelMaxUs - m_dRelMinUs;
    if (spread < 0.0)
        spread = 0.0;
    if (spread > double(CAP_US))
        spread = double(CAP_US);
    m_uHoldUs.store(uint32_t(spread));
}

uint32_t srt::SrtlaReorderHold::holdUs(int tsbpd_delay_ms) const
{
    uint32_t hold = m_uHoldUs.load();
    if (hold == 0)
        return 0;

    const uint32_t half_latency_us = tsbpd_delay_ms > 0 ? uint32_t(tsbpd_delay_ms) * 500 : 0;
    if (half_latency_us > 0 && hold > half_latency_us)
        hold = half_latency_us;
    return hold;
}
