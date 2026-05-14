// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 *
 */
/**
 * @file SamplePLL.h
 * @ingroup modem
 * @file SamplePLL.cpp
 * @ingroup modem_fw
 */
#if !defined(__SAMPLE_PLL_H__)
#define __SAMPLE_PLL_H__

#include "Defines.h"

#include <cstdint>
#include <string>

namespace modem
{
    // ---------------------------------------------------------------------------
    //  Class Declaration
    // ---------------------------------------------------------------------------

    /**
     * @brief Implements a second-order digital PLL for sample clock recovery.
     * @ingroup modem
     * 
     * This PLL tracks frequency and phase errors in the received sample stream,
     * allowing long-duration demodulation without sync loss due to oscillator drift.
     */
    class DSP_FW_API SamplePLL {
    public:
        /**
         * @brief Initializes a new instance of SamplePLL.
         * @param nominalSampleRate Nominal sample rate in Hz (e.g., 4000000).
         */
        SamplePLL(double nominalSampleRate);
        ~SamplePLL();

        /**
         * @brief Update PLL with timing error measurement.
         * 
         * Called after each frame sync detection. The phase error indicates
         * how much the actual timing has drifted from expected.
         * 
         * @param phaseErrorSamples Timing error in samples (positive = early, negative = late).
         */
        void update(int16_t phaseErrorSamples);

        /**
         * @brief Get the current frequency correction in PPM (parts per million).
         * @return double Frequency offset in PPM (e.g., +5.0 for 5 ppm faster).
         */
        double getFreqOffsetPPM() const { return m_freqOffsetPPM; }

        /**
         * @brief Get the current phase correction in samples.
         * @return int32_t Phase correction to apply to sample timing.
         */
        int32_t getPhaseCorrection() const { return m_phaseCorr; }

        /**
         * @brief Check if PLL is currently locked.
         * @return bool True if timing error is within lock threshold.
         */
        bool isLocked() const { return m_locked; }

        /**
         * @brief Reset PLL state (call on loss of sync).
         */
        void reset();

        /**
         * @brief Enable/disable PLL adaptation.
         * @param enable True to enable PLL, false to freeze.
         */
        void setEnabled(bool enable) { m_enabled = enable; }

        /**
         * @brief Set PLL loop gains (tuning for response speed).
         * @param kp Proportional gain (typical: 0.001 to 0.01).
         * @param ki Integral gain (typical: 0.0001 to 0.001).
         */
        void setGains(double kp, double ki);

        /**
         * @brief Get diagnostic information for logging/debugging.
         * @return std::string Formatted PLL state for debug output.
         */
        std::string getDiagnostics() const;

    private:
        // PLL state
        double m_nominalSampleRate;    // Nominal rate (e.g., 4 MHz)
        double m_freqOffsetPPM;        // Estimated frequency offset (PPM)
        int32_t m_phaseCorr;           // Current phase correction (samples)

        // Loop filter coefficients (2nd order)
        double m_kp;                   // Proportional gain
        double m_ki;                   // Integral gain
        
        // Integrator state
        double m_integralState;        // Accumulated integral (for lock range)

        // Configuration
        bool m_enabled;
        uint32_t m_updateCount;        // Frame counter for diagnostics

        // Lock detection
        int32_t m_lockThreshold;       // Max phase error for lock (samples)
        bool m_locked;                 // PLL currently locked
    };
} // namespace modem

#endif // __SAMPLE_PLL_H__
