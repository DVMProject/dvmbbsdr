// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * Copyright (C) 2026 by Jonathan Naylor G4KLX
 * Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 */
#include "radio/FDUDC.h"

#include <cmath>

using namespace radio;

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

#ifndef M_PIf32
#define M_PIf32 3.141592653589793f
#endif

// ---------------------------------------------------------------------------
//  Global Functions
// ---------------------------------------------------------------------------

/**
 * @brief Helper function to compute the sinc function, which is defined as sin(pi * x) / (pi * x) for x != 0, and 1 
 *  for x = 0. This function is commonly used in signal processing for interpolation and resampling. The implementation 
 *  includes a check for the zero case to avoid division by zero, and returns the appropriate value based on the input.
 * @param v The input value for which to compute the sinc function.
 * @returns float The computed sinc value for the input v.
 */
static float sinc(float v)
{
    if (v == 0.0f)
        return 1.0f;
    return std::sin(v) / v;
}

/**
 * @brief Helper function to compute a Hann window value for a given index and total length. The Hann window is a 
 *  commonly used window function in signal processing to reduce spectral leakage when performing Fourier transforms.
 *  This function computes the Hann window value based on the index and total length, using the standard formula for the Hann window.
 * @param i The index for which to compute the Hann window value.
 * @param length The total length of the window, which is used to normalize the index and compute the window value.
 * @returns float The computed Hann window value for the given index and length.
 */
static float hannWindow(size_t i, size_t length)
{
    return 0.5f - 0.5f * std::cos((0.5f + static_cast<float>(i)) / static_cast<float>(length) * (M_PIf32 * 2.0f));
}

/**
 * @brief Helper function to compute a windowed sinc filter coefficient for a given index, total length, and cutoff 
 *  frequency. This function combines the sinc function with a Hann window to create a windowed sinc filter coefficient, 
 *  which is used in the resampling process of the FDUDC. The cutoff frequency is normalized as a fraction of the 
 *  resampling frequency, and the function computes the appropriate sinc value and applies the Hann window to produce 
 *  the final filter coefficient.
 * @param i The index for which to compute the windowed sinc coefficient.
 * @param length The total length of the filter, which is used to normalize the index and compute the window value.
 * @param cutoff The normalized cutoff frequency for the sinc function, as a fraction of the resampling frequency (0.0 to 0.5).
 * @returns float The computed windowed sinc filter coefficient for the given index, length, and cutoff frequency.
 */
static float windowedSinc(size_t i, size_t length, float cutoff)
{
    return sinc(cutoff * (static_cast<float>(i) - 0.5f * static_cast<float>(length))) * hannWindow(i, length);
}

/**
 * @brief Helper function to generate a sine table for a given frequency numerator and denominator. This function creates a 
 *  table of complex sine values that can be used for efficient mixing in the FDUDC. The frequency is determined by the 
 *  ratio of the numerator to the denominator, and the table is generated with a size equal to the denominator to cover 
 *  one full cycle of the sine wave. The generated sine values are stored in a vector of complex floats, where the real 
 *  part represents the cosine component and the imaginary part represents the sine component of the wave.
 * @param table Reference to a vector of complex floats that will be filled with the generated sine values.
 * @param freqNum The numerator of the frequency ratio used to determine the frequency of the sine wave.
 * @param freqDen The denominator of the frequency ratio used to determine the frequency of the sine wave. The size of 
 *  the generated table will be equal to this value, covering one full cycle of the sine wave.
 * @returns void This function does not return a value, but fills the provided table with the generated sine values 
 *  based on the specified frequency ratio.
 */
static void makeSineTable(std::vector<std::complex<float>>& table, int freqNum, unsigned freqDen)
{
    const float freq = static_cast<float>(freqNum) / static_cast<float>(freqDen) * (M_PIf32 * 2.0f);
    table.resize(freqDen);
    for (size_t i = 0U; i < static_cast<size_t>(freqDen); i++) {
        table[i] = std::polar(1.0f, freq * static_cast<float>(i));
    }
}

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the FDUDC class. */

FDUDC::FDUDC(uint32_t resampNum, uint32_t resampDen, int rxIfNum, uint32_t rxIfDen,
    int txIfNum, uint32_t txIfDen, uint32_t length, float cutoff) :
    m_resampNum(resampNum),
    m_resampDen(resampDen),
    m_p(0U),
    m_i(0U),
    m_ddcI(0U),
    m_ducI(0U),
    m_ducIn(0.0f, 0.0f),
    m_taps(),
    m_in(),
    m_out(),
    m_ddcSine(),
    m_ducSine()
{
    const size_t approxLen = static_cast<size_t>(resampDen) * static_cast<size_t>(length);
    const size_t branches = resampNum;
    const size_t branchLen = (approxLen + branches / 2U) / branches;
    const size_t totalLen = branchLen * branches;

    m_taps.resize(totalLen);

    // generate the windowed sinc filter coefficients for the resampling process, which are used to perform the 
    // fractional sample rate conversion in the FDUDC, the cutoff frequency is normalized based on the resampling 
    // denominator to ensure that the filter is designed appropriately for the desired resampling ratio, and the 
    // coefficients are scaled to ensure that the overall gain of the filter is correct for the resampling process, 
    // which helps to maintain signal integrity and minimize distortion during the conversion
    const float sincCutoff = (cutoff * M_PIf32) / static_cast<float>(resampDen);
    float sum = 0.0f;
    for (size_t i = 0U; i < totalLen; i++) {
        const float v = windowedSinc(i, totalLen, sincCutoff);
        m_taps[i] = v;
        sum += v;
    }

    const float scaling = static_cast<float>(branches) / sum;
    for (size_t i = 0U; i < totalLen; i++) {
        m_taps[i] *= scaling;
    }

    m_in.resize(branchLen * 2U);
    m_out.resize(branchLen * 2U);

    makeSineTable(m_ddcSine, -rxIfNum, rxIfDen);
    makeSineTable(m_ducSine, txIfNum, txIfDen);
}

/* Finalizes an instance of the FDUDC class. */

FDUDC::~FDUDC() = default;

/* Processes a buffer of complex samples through the FDUDC, applying the specified sample processing function. */

void FDUDC::process(std::vector<std::complex<float>>& buffer,
    std::function<std::complex<float>(std::complex<float>)> processSample)
{
    const size_t branchLen = m_in.size() / 2U;
    const float ducScaling = static_cast<float>(m_resampDen) / static_cast<float>(m_resampNum);

    // process each sample in the input buffer through the FDUDC, applying the configured resampling and frequency shifting
    // the input samples are mixed with the DDC sine wave to shift the frequency, then the resampling is performed using the
    // windowed sinc filter coefficients, and the output is mixed with the DUC sine wave to shift the frequency back up, 
    // while also applying the provided sample processing function to allow for additional custom sample-level processing
    for (auto& sample : buffer) {
        m_in[m_i] = m_in[m_i + branchLen] = sample * m_ddcSine[m_ddcI];
        if (++m_ddcI >= m_ddcSine.size())
            m_ddcI = 0U;

        m_p += m_resampNum;
        while (m_p >= m_resampDen) {
            m_p -= m_resampDen;

            size_t p = static_cast<size_t>(m_p);
            auto windowIn = &m_in[m_i + 1U];
            auto windowOut = &m_out[m_i + 1U];

            std::complex<float> ddcOut(0.0f, 0.0f);
            for (size_t i = 0U; i < branchLen; i++) {
                const float tap = m_taps[p];
                ddcOut += windowIn[i] * tap;
                windowOut[i] += m_ducIn * tap;
                p += static_cast<size_t>(m_resampNum);
            }

            m_ducIn = ducScaling * processSample(ddcOut);
        }

        sample = (m_out[m_i] + m_out[m_i + branchLen]) * m_ducSine[m_ducI];
        m_out[m_i] = std::complex<float>(0.0f, 0.0f);
        m_out[m_i + branchLen] = std::complex<float>(0.0f, 0.0f);

        if (++m_ducI >= m_ducSine.size())
            m_ducI = 0U;
        if (++m_i >= branchLen)
            m_i = 0U;
    }
}
