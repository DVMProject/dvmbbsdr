// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 *
 */
#include "modem/SamplePLL.h"
#include "common/Log.h"

#include <cmath>
#include <sstream>

using namespace modem;

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of SamplePLL. */

SamplePLL::SamplePLL(double nominalSampleRate) :
    m_nominalSampleRate(nominalSampleRate),
    m_freqOffsetPPM(0.0),
    m_phaseCorr(0),
    m_kp(0.05),                // Proportional gain (tuned for 24 kHz sample rate)
    m_ki(0.005),               // Integral gain (accumulates timing errors for frequency tracking)
    m_integralState(0.0),
    m_enabled(true),
    m_updateCount(0U),
    m_lockThreshold(2),        // Lock if |phaseError| < 2 samples
    m_locked(false)
{
    /* stub */
}

/* Finalizes a instance of SamplePLL. */

SamplePLL::~SamplePLL()
{
    /* stub */
}

/* Update PLL with timing error measurement. */

void SamplePLL::update(int16_t phaseErrorSamples)
{
    if (!m_enabled)
        return;

    m_updateCount++;

    // Check lock status
    bool wasLocked = m_locked;
    m_locked = (std::abs(phaseErrorSamples) <= m_lockThreshold);

    if (!wasLocked && m_locked) {
        LogInfoEx(LOG_HOST, "SamplePLL locked at %+.2f ppm", m_freqOffsetPPM);
    }
    else if (wasLocked && !m_locked) {
        LogWarning(LOG_HOST, "SamplePLL lost lock");
    }

    // Proportional term: direct phase correction
    double pError = static_cast<double>(phaseErrorSamples);
    int32_t pTerm = static_cast<int32_t>(m_kp * pError);

    // Integral term: accumulated error → frequency correction
    // Accumulate phase error over multiple frames
    m_integralState += pError;

    // Limit integral windup (prevent unbounded accumulation)
    const double integralMax = 100.0;  // ~25 ppm at 4 MHz
    if (m_integralState > integralMax)
        m_integralState = integralMax;
    if (m_integralState < -integralMax)
        m_integralState = -integralMax;

    double iTerm = m_ki * m_integralState;

    // Combine P and I: iTerm becomes frequency correction (PPM)
    m_freqOffsetPPM = iTerm;

    // Phase correction (applied next symbol detection)
    m_phaseCorr = pTerm;

    // Diagnostics every 100 frames
    if ((m_updateCount % 100U) == 0U) {
        LogDebugEx(LOG_HOST, "SamplePLL::update()", "SamplePLL, phaseErr=%d freqOff=%+.2f ppm locked=%s",
            phaseErrorSamples, m_freqOffsetPPM, m_locked ? "yes" : "no");
    }
}

/* Reset PLL state. */

void SamplePLL::reset()
{
    m_freqOffsetPPM = 0.0;
    m_phaseCorr = 0;
    m_integralState = 0.0;
    m_locked = false;
    m_updateCount = 0U;
    
    LogInfoEx(LOG_HOST, "SamplePLL reset");
}

/* Set PLL loop gains. */

void SamplePLL::setGains(double kp, double ki)
{
    m_kp = kp;
    m_ki = ki;
    LogInfoEx(LOG_HOST, "SamplePLL gains, Kp=%.6f, Ki=%.6f", kp, ki);
}

/* Get diagnostic information. */

std::string SamplePLL::getDiagnostics() const
{
    std::ostringstream ss;
    ss << "SamplePLL, freqOff=" << m_freqOffsetPPM << " ppm, "
       << "phaseCorr=" << m_phaseCorr << " samples, "
       << "locked=" << (m_locked ? "yes" : "no") << ", "
       << "updates=" << m_updateCount;
    return ss.str();
}
