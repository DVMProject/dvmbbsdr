// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * Copyright (C) 2026 by Jonathan Naylor G4KLX
 * Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 */
/**
 * @file FDUDC.h
 * @ingroup modem_fw
 * @file FDUDC.cpp
 * @ingroup modem_fw
 *
 * @brief Digital up and down conversion with fractional sample rate conversion.
 */
#if !defined(___FDUDC_H__)
#define ___FDUDC_H__

#include "Defines.h"

#include <complex>
#include <functional>
#include <vector>

namespace radio
{
    /**
     * @brief Implements the FDUDC class, which performs digital up and down conversion with fractional sample 
     *  rate conversion. This class is used to resample and shift the frequency of complex baseband samples for 
     *  transmission and reception in the SDR runtime. The FDUDC class uses a polyphase filter bank for efficient 
     *  resampling, and supports configurable resampling ratios and filter parameters.
     */
    class DSP_FW_API FDUDC {
    public:
        /**
         * @brief Initializes a new instance of the FDUDC class.
         * @param resampNum Resampling numerator for the fractional sample rate conversion.
         * @param resampDen Resampling denominator for the fractional sample rate conversion.
         * @param rxIfNum RX intermediate frequency numerator for the downconversion.
         * @param rxIfDen RX intermediate frequency denominator for the downconversion.
         * @param txIfNum TX intermediate frequency numerator for the upconversion.
         * @param txIfDen TX intermediate frequency denominator for the upconversion.
         * @param length Length of the filter used for resampling, in number of taps.
         * @param cutoff Normalized cutoff frequency for the resampling filter, as a fraction of the resampling frequency (0.0 to 0.5).
         */
        FDUDC(uint32_t resampNum, uint32_t resampDen, int32_t rxIfNum, uint32_t rxIfDen, int32_t txIfNum,
            uint32_t txIfDen, uint32_t length = 9U, float cutoff = 0.45f);
        /**
         * @brief Finalizes an instance of the FDUDC class.
         */
        ~FDUDC();

        /**
         * @brief Processes a buffer of complex samples through the FDUDC, applying the configured resampling and 
         *  frequency shifting. The provided processSample function is applied to each sample after processing, 
         *  allowing for additional sample-level processing if needed. The method modifies the input buffer in place 
         *  with the processed samples.
         * @param buffer Reference to a vector of complex float samples to be processed. The samples are modified in 
         *  place with the output of the FDUDC processing.
         * @param processSample Function that takes a complex float sample and returns a processed complex float sample. 
         *  This function is applied to each sample after the FDUDC processing, allowing for additional custom
         *  sample-level processing.
         */
        void process(std::vector<std::complex<float>>& buffer, std::function<std::complex<float>(std::complex<float>)> processSample);

    private:
        uint32_t m_resampNum;
        uint32_t m_resampDen;

        uint32_t m_p;
        uint32_t m_i;
        uint32_t m_ddcI;
        uint32_t m_ducI;

        std::complex<float> m_ducIn;

        std::vector<float> m_taps;
        std::vector<std::complex<float>> m_in;
        std::vector<std::complex<float>> m_out;
        std::vector<std::complex<float>> m_ddcSine;
        std::vector<std::complex<float>> m_ducSine;
    };
}

#endif // ___FDUDC_H__
