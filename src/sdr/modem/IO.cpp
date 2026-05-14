// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2015,2016,2017 Jonathan Naylor, G4KLX
 *  Copyright (C) 2015 Jim Mclaughlin, KI6ZUM
 *  Copyright (C) 2016 Colin Durbridge, G4EML
 *  Copyright (C) 2017-2026 Bryan Biedenkapp, N2PLL
 *
 */
#include "modem/IO.h"
#include "modem/Modem.h"
#include "radio/RadioManager.h"
#include "common/Log.h"

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <chrono>

using namespace modem;

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

#define RX_RF_REPORT_INTERVAL_MS 1000U
#define RX_RF_IDLE_INTERVAL_MS 500U
#define TX_FRAME_SAMPLES 720U      // 30 ms at 24 kHz modem sample rate
#define TX_FRAME_SLEEP_US 30000U   // pace one TX frame in real

// Generated using rcosdesign(0.2, 8, 5, 'sqrt') in MATLAB
static q15_t RRC_0_2_FILTER[] = {
    401, 104, -340, -731, -847, -553, 112, 909, 1472, 1450, 683, -675, -2144, -3040, -2706, -770, 2667, 6995,
    11237, 14331, 15464, 14331, 11237, 6995, 2667, -770, -2706, -3040, -2144, -675, 683, 1450, 1472, 909, 112,
    -553, -847, -731, -340, 104, 401, 0 };
const uint16_t RRC_0_2_FILTER_LEN = 42U;

// One symbol boxcar filter
#if defined(P25_RX_NORMAL_BOXCAR)
static q15_t BOXCAR_5_FILTER[] = { 12000, 12000, 12000, 12000, 12000, 0 };
#endif
#if defined(P25_RX_NARROW_BOXCAR)
static q15_t BOXCAR_5_FILTER[] = { 8000, 8000, 8000, 8000, 8000, 0 };
#endif
const uint16_t BOXCAR_5_FILTER_LEN = 6U;

#if defined(NXDN_BOXCAR_FILTER)
// One symbol boxcar filter
static q15_t BOXCAR_10_FILTER[] = { 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000 };
const uint16_t BOXCAR_10_FILTER_LEN = 10U;
#else
// Generated using rcosdesign(0.2, 8, 10, 'sqrt') in MATLAB
static q15_t NXDN_0_2_FILTER[] = {
    284, 198, 73, -78, -240, -393, -517, -590, -599, -533, -391, -181, 79, 364, 643, 880, 1041, 1097, 1026, 819,
    483, 39, -477, -1016, -1516, -1915, -2150, -2164, -1914, -1375, -545, 557, 1886, 3376, 4946, 6502, 7946, 9184,
    10134, 10731, 10935, 10731, 10134, 9184, 7946, 6502, 4946, 3376, 1886, 557, -545, -1375, -1914, -2164, -2150,
    -1915, -1516, -1016, -477, 39, 483, 819, 1026, 1097, 1041, 880, 643, 364, 79, -181, -391, -533, -599, -590,
    -517, -393, -240, -78, 73, 198, 284, 0
};
const uint16_t NXDN_0_2_FILTER_LEN = 82U;

static q15_t NXDN_ISINC_FILTER[] = {
    790, -1085, -1073, -553, 747, 2341, 3156, 2152, -893, -4915, -7834, -7536, -3102, 4441, 12354, 17394, 17394,
    12354, 4441, -3102, -7536, -7834, -4915, -893, 2152, 3156, 2341, 747, -553, -1073, -1085, 790
};
const uint16_t NXDN_ISINC_FILTER_LEN = 32U;
#endif

// Generated using [b, a] = butter(1, 0.001) in MATLAB
static q31_t DC_FILTER[] = { 3367972, 0, 3367972, 0, 2140747704, 0 }; // {b0, 0, b1, b2, -a1, -a2}
const uint32_t DC_FILTER_STAGES = 1U; // One Biquad stage

// ---------------------------------------------------------------------------
//  Global Functions
// ---------------------------------------------------------------------------

/**
 * @brief Helper to get current monotonic time in milliseconds.
 * @returns Current monotonic time in milliseconds.
 */
uint64_t monotonicMs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the IO class. */

IO::IO(modem::Modem* modem, bool debug) :
    m_modem(modem),
    m_started(false),
    m_rxBuffer(RX_RINGBUFFER_SIZE),
    m_txBuffer(TX_RINGBUFFER_SIZE),
    m_rssiBuffer(RX_RINGBUFFER_SIZE),
    m_rrc_0_2_Filter(),
    m_boxcar_5_Filter(),
    m_dcFilter(),
    m_rrc_0_2_State(),
    m_boxcar_5_State(),
    m_dcState(),
    m_pttInvert(false),
    m_rxLevel(128 * 128),
    m_rxInvert(false),
    m_txInvert(false),
    m_cwIdTXLevel(128 * 128),
    m_dmrTXLevel(128 * 128),
    m_p25TXLevel(128 * 128),
    m_nxdnTXLevel(128 * 128),
    m_detect(false),
    m_adcOverflow(0U),
    m_dacOverflow(0U),
    m_watchdog(0U),
    m_lockout(false),
    m_rxFrequency(DEFAULT_FREQUENCY),
    m_txFrequency(DEFAULT_FREQUENCY),
    m_rfPower(0U),
    m_threadTx(),
    m_txLock(),
    m_threadRx(),
    m_rxLock(),
    m_threadStatus(),
    m_audioBufTx(),
    m_audioBufRx(),
    m_abort(false),
    m_rxRfActive(false),
    m_rxRfLastDataMs(0U),
    m_rxRfLastReportMs(0U),
    m_rxRfBytes(0U),
    m_rxRfBursts(0U),
    m_cosPrev(false),
    m_cosInt(false),
    m_pttPrev(false),
    m_ptt(false),
    m_dmrModeToggle(false),
    m_dmrMode(false),
    m_p25ModeToggle(false),
    m_p25Mode(false),
    m_nxdnModeToggle(false),
    m_nxdnMode(false),
    m_debug(debug)
{
    ::memset(m_rrc_0_2_State, 0x00U, 70U * sizeof(q15_t));
    ::memset(m_boxcar_5_State, 0x00U, 30U * sizeof(q15_t));

    ::memset(m_dcState, 0x00U, 4U * sizeof(q31_t));

    m_rrc_0_2_Filter.numTaps = RRC_0_2_FILTER_LEN;
    m_rrc_0_2_Filter.pState = m_rrc_0_2_State;
    m_rrc_0_2_Filter.pCoeffs = RRC_0_2_FILTER;

    m_boxcar_5_Filter.numTaps = BOXCAR_5_FILTER_LEN;
    m_boxcar_5_Filter.pState = m_boxcar_5_State;
    m_boxcar_5_Filter.pCoeffs = BOXCAR_5_FILTER;

#if NXDN_BOXCAR_FILTER
    ::memset(m_boxcar_10_State, 0x00U, 40U * sizeof(q15_t));
    
    m_boxcar_10_Filter.numTaps = BOXCAR10_FILTER_LEN;
    m_boxcar_10_Filter.pState  = m_boxcar_10_State;
    m_boxcar_10_Filter.pCoeffs = BOXCAR10_FILTER;
#else
    ::memset(m_nxdn_0_2_State, 0x00U, 110U * sizeof(q15_t));
    ::memset(m_nxdn_ISinc_State, 0x00U, 60U * sizeof(q15_t));

    m_nxdn_0_2_Filter.numTaps = NXDN_0_2_FILTER_LEN;
    m_nxdn_0_2_Filter.pState  = m_nxdn_0_2_State;
    m_nxdn_0_2_Filter.pCoeffs = NXDN_0_2_FILTER;
    
    m_nxdn_ISinc_Filter.numTaps = NXDN_ISINC_FILTER_LEN;
    m_nxdn_ISinc_Filter.pState  = m_nxdn_ISinc_State;
    m_nxdn_ISinc_Filter.pCoeffs = NXDN_ISINC_FILTER;
#endif

    m_dcFilter.numStages = DC_FILTER_STAGES;
    m_dcFilter.pState = m_dcState;
    m_dcFilter.pCoeffs = DC_FILTER;
    m_dcFilter.postShift = 0;
}

/* Finalizes a instance of the IO class. */

IO::~IO()
{
    m_abort = true;

    if (m_threadTx) {
        ::pthread_join(m_threadTx, NULL);
    }

    if (m_threadRx) {
        ::pthread_join(m_threadRx, NULL);
    }

    if (m_threadStatus) {
        ::pthread_join(m_threadStatus, NULL);
    }

    ::pthread_mutex_destroy(&m_txLock);
    ::pthread_mutex_destroy(&m_rxLock);
}

/* Starts air interface sampler. */

void IO::start()
{
    if (m_started)
        return;

    startInt();

    m_started = true;

    setMode();
}

/* Process samples from air interface. */

void IO::process()
{
    if (m_started) {
        // Two seconds timeout
        if (m_watchdog >= 48000U) {
            if (m_modem->m_modemState == STATE_DMR || m_modem->m_modemState == STATE_P25 || m_modem->m_modemState == STATE_NXDN) {
                if (m_modem->m_modemState == STATE_DMR && m_modem->m_tx)
                    m_modem->m_dmrTX.setStart(false);
                m_modem->m_modemState = STATE_IDLE;
                setMode();
            }

            m_watchdog = 0U;
        }
    }
    else {
        return;
    }

    // use the COS line to lockout the modem
    if (m_modem->m_cosLockoutEnable) {
        m_lockout = getCOSInt();
    }

    // Switch off the transmitter only when there is no queued or staged TX data.
    // The TX ring can be empty while interrupt() is still holding unsent staged samples.
    bool hasPendingTx = false;
    ::pthread_mutex_lock(&m_txLock);
    if (m_txBuffer.getData() > 0U) {
        hasPendingTx = true;
    } else {
        hasPendingTx = !m_audioBufTx.empty();
    }
    ::pthread_mutex_unlock(&m_txLock);

    if (!hasPendingTx && m_modem->m_tx) {
        m_modem->m_tx = false;
        m_modem->setModemTxActive(false);

        if (m_debug)
            ::LogDebugEx(LOG_SDR, "IO::process()", "no Tx data, TX OFF");
        setPTTInt(m_pttInvert ? true : false);
    }

    if (m_rxBuffer.getData() >= RX_BLOCK_SIZE) {
        q15_t samples[RX_BLOCK_SIZE];
        uint8_t control[RX_BLOCK_SIZE];
        uint16_t rssi[RX_BLOCK_SIZE];

        for (uint16_t i = 0U; i < RX_BLOCK_SIZE; i++) {
            int16_t sample = 0;
            m_rxBuffer.get(sample, control[i]);
            m_rssiBuffer.get(rssi[i]);

            // Detect ADC overflow.
            if (m_detect && (sample == 32767 || sample == -32768))
                m_adcOverflow++;

            q15_t res1 = q15_t(sample);
            q31_t res2 = res1 * m_rxLevel;
            samples[i] = q15_t(__SSAT((res2 >> 15), 16));
        }

        if (m_lockout)
            return;

        q15_t dcSamples[RX_BLOCK_SIZE];
        if (m_modem->m_dcBlockerEnable) {
            q31_t q31Samples[RX_BLOCK_SIZE];

            ::arm_q15_to_q31(samples, q31Samples, RX_BLOCK_SIZE);

            q31_t dcValues[RX_BLOCK_SIZE];
            ::arm_biquad_cascade_df1_q31(&m_dcFilter, q31Samples, dcValues, RX_BLOCK_SIZE);

            q31_t dcLevel = 0;
            for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
                dcLevel += dcValues[i];
            dcLevel /= RX_BLOCK_SIZE;

            q15_t offset = q15_t(__SSAT((dcLevel >> 16), 16));;

            for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
                dcSamples[i] = samples[i] - offset;
        }

        /** Idle Modem State */
        if (m_modem->m_modemState == STATE_IDLE) {
            /** Project 25 */
            if (m_modem->m_p25Enable) {
                q15_t c4fmSamples[RX_BLOCK_SIZE];
                if (m_modem->m_dcBlockerEnable) {
                    ::arm_fir_fast_q15(&m_boxcar_5_Filter, dcSamples, c4fmSamples, RX_BLOCK_SIZE);
                }
                else {
                    ::arm_fir_fast_q15(&m_boxcar_5_Filter, samples, c4fmSamples, RX_BLOCK_SIZE);
                }

                m_modem->m_p25RX.samples(c4fmSamples, rssi, RX_BLOCK_SIZE);
            }

            /** Digital Mobile Radio */
            if (m_modem->m_dmrEnable) {
                q15_t c4fmSamples[RX_BLOCK_SIZE];
                ::arm_fir_fast_q15(&m_rrc_0_2_Filter, samples, c4fmSamples, RX_BLOCK_SIZE);

                if (m_modem->m_dmrEnable) {
                    if (m_modem->m_duplex)
                        m_modem->m_dmrIdleRX.samples(c4fmSamples, RX_BLOCK_SIZE);
                    else
                        m_modem->m_dmrDMORX.samples(c4fmSamples, rssi, RX_BLOCK_SIZE);
                }
            }

            /** Next Generation Digital Narrowband */
            if (m_modem->m_nxdnEnable) {
                q15_t c4fmSamples[RX_BLOCK_SIZE];
#if NXDN_BOXCAR_FILTER
                if (m_modem->m_dcBlockerEnable) {
                    ::arm_fir_fast_q15(&m_boxcar_10_Filter, dcSamples, c4fmSamples, RX_BLOCK_SIZE);
                }
                else {
                    ::arm_fir_fast_q15(&m_boxcar_10_Filter, samples, c4fmSamples, RX_BLOCK_SIZE);
                }
#else
                q15_t c4fmRCSamples[RX_BLOCK_SIZE];
                if (m_modem->m_dcBlockerEnable) {
                    ::arm_fir_fast_q15(&m_nxdn_0_2_Filter, dcSamples, c4fmRCSamples, RX_BLOCK_SIZE);
                }
                else {
                    ::arm_fir_fast_q15(&m_nxdn_0_2_Filter, samples, c4fmRCSamples, RX_BLOCK_SIZE);
                }

                ::arm_fir_fast_q15(&m_nxdn_ISinc_Filter, c4fmRCSamples, c4fmSamples, RX_BLOCK_SIZE);
#endif
                m_modem->m_nxdnRX.samples(c4fmSamples, rssi, RX_BLOCK_SIZE);
            }
        }
        else if (m_modem->m_modemState == STATE_DMR) {        // DMR State
            /** Digital Mobile Radio */
            if (m_modem->m_dmrEnable) {
                q15_t c4fmSamples[RX_BLOCK_SIZE];
                ::arm_fir_fast_q15(&m_rrc_0_2_Filter, samples, c4fmSamples, RX_BLOCK_SIZE);

                if (m_modem->m_duplex) {
                    // If the transmitter isn't on, use the DMR idle RX to detect the wakeup CSBKs
                    if (m_modem->m_tx)
                        m_modem->m_dmrRX.samples(c4fmSamples, rssi, control, RX_BLOCK_SIZE);
                    else
                        m_modem->m_dmrIdleRX.samples(c4fmSamples, RX_BLOCK_SIZE);
                }
                else {
                    m_modem->m_dmrDMORX.samples(c4fmSamples, rssi, RX_BLOCK_SIZE);
                }
            }
        }
        else if (m_modem->m_modemState == STATE_P25) {        // P25 State
            /** Project 25 */
            if (m_modem->m_p25Enable) {
                q15_t c4fmSamples[RX_BLOCK_SIZE];
                if (m_modem->m_dcBlockerEnable) {
                    ::arm_fir_fast_q15(&m_boxcar_5_Filter, dcSamples, c4fmSamples, RX_BLOCK_SIZE);
                }
                else {
                    ::arm_fir_fast_q15(&m_boxcar_5_Filter, samples, c4fmSamples, RX_BLOCK_SIZE);
                }

                m_modem->m_p25RX.samples(c4fmSamples, rssi, RX_BLOCK_SIZE);
            }
        }
        else if (m_modem->m_modemState == STATE_NXDN) {       // NXDN State
            /** Next Generation Digital Narrowband */
            if (m_modem->m_nxdnEnable) {
                q15_t c4fmSamples[RX_BLOCK_SIZE];
#if NXDN_BOXCAR_FILTER
                if (m_modem->m_dcBlockerEnable) {
                    ::arm_fir_fast_q15(&m_boxcar_10_Filter, dcSamples, c4fmSamples, RX_BLOCK_SIZE);
                }
                else {
                    ::arm_fir_fast_q15(&m_boxcar_10_Filter, samples, c4fmSamples, RX_BLOCK_SIZE);
                }
#else
                q15_t c4fmRCSamples[RX_BLOCK_SIZE];
                if (m_modem->m_dcBlockerEnable) {
                    ::arm_fir_fast_q15(&m_nxdn_0_2_Filter, dcSamples, c4fmRCSamples, RX_BLOCK_SIZE);
                }
                else {
                    ::arm_fir_fast_q15(&m_nxdn_0_2_Filter, samples, c4fmRCSamples, RX_BLOCK_SIZE);
                }

                ::arm_fir_fast_q15(&m_nxdn_ISinc_Filter, c4fmRCSamples, c4fmSamples, RX_BLOCK_SIZE);
#endif
                m_modem->m_nxdnRX.samples(c4fmSamples, rssi, RX_BLOCK_SIZE);
            }
        }
        else if (m_modem->m_modemState == STATE_RSSI_CAL) {
            m_modem->m_calRSSI.samples(rssi, RX_BLOCK_SIZE);
        }
    }
}

/* Write samples to air interface. */

void IO::write(DVM_STATE mode, q15_t* samples, uint16_t length, const uint8_t* control)
{
    if (!m_started)
        return;

    if (m_lockout)
        return;

    // Switch the transmitter on if needed
    if (!m_modem->m_tx) {
        m_modem->m_tx = true;
        m_modem->setModemTxActive(true);

        if (m_debug)
            ::LogDebugEx(LOG_SDR, "IO::write()", "TX ON, first sample written");
        setPTTInt(m_pttInvert ? false : true);
    }

    q15_t txLevel = 0;
    switch (mode) {
    case STATE_DMR:
        txLevel = m_dmrTXLevel;
        break;
    case STATE_P25:
        txLevel = m_p25TXLevel;
        break;
    case STATE_NXDN:
        txLevel = m_nxdnTXLevel;
        break;
    default:
        txLevel = m_cwIdTXLevel;
        break;
    }

    // TX ring is consumed by the dedicated TX thread; writes must be serialized
    // with reads to prevent ring index corruption under load.
    ::pthread_mutex_lock(&m_txLock);
    for (uint16_t i = 0U; i < length; i++) {
        q31_t res1 = samples[i] * txLevel;
        q15_t res2 = q15_t(__SSAT((res1 >> 15), 16));
        int16_t res3 = static_cast<int16_t>(res2);

        // Detect DAC overflow.
        if (res3 == 32767 || res3 == -32768)
            m_dacOverflow++;

        if (control == NULL)
            m_txBuffer.put(res3, MARK_NONE);
        else
            m_txBuffer.put(res3, control[i]);
    }
    ::pthread_mutex_unlock(&m_txLock);
}

/* Helper to get how much space the transmit ring buffer has for samples. */

uint16_t IO::getSpace() const
{
    return m_txBuffer.getSpace();
}

/* */

void IO::setDecode(bool dcd)
{
    if (dcd != m_modem->m_dcd)
        setCOSInt(dcd ? true : false);

    m_modem->m_dcd = dcd;
}

/* */

void IO::setADCDetection(bool detect)
{
    m_detect = detect;
}

/* Helper to set the modem air interface state. */

void IO::setMode()
{
    DVM_STATE relativeState = m_modem->m_modemState;

    if (m_modem->m_serial.isCalState(m_modem->m_modemState)) {
        relativeState = m_modem->m_serial.calRelativeState(m_modem->m_modemState);
    }

    m_modem->writeDebug("IO::setMode() setting modem state", m_modem->m_modemState, relativeState);

    m_modem->writeDebug("IO::setMode() setting lights", relativeState == STATE_DMR, relativeState == STATE_P25, relativeState == STATE_NXDN);
    setDMRInt(relativeState == STATE_DMR);
    setP25Int(relativeState == STATE_P25);
    setNXDNInt(relativeState == STATE_NXDN);
}

/* Helper to assert or deassert radio PTT. */

void IO::setTransmit()
{
    // Assert TX when requested by protocol engines. Deassert is handled by process()
    // once there are no pending TX samples left.
    if (m_modem->m_tx)
        return;

    m_modem->m_tx = true;
    m_modem->setModemTxActive(true);

    if (m_debug)
        ::LogDebugEx(LOG_SDR, "IO::setTransmit()", "TX ON");
    setPTTInt(m_pttInvert ? false : true);
}

/* Sets various air interface parameters. */

void IO::setParameters(bool rxInvert, bool txInvert, bool pttInvert, uint8_t rxLevel, uint8_t cwIdTXLevel, uint8_t dmrTXLevel,
                       uint8_t p25TXLevel, uint8_t nxdnTXLevel)
{
    m_pttInvert = pttInvert;
    m_rxInvert = rxInvert;
    m_txInvert = txInvert;

    m_rxLevel = q15_t(rxLevel * 128);
    m_cwIdTXLevel = q15_t(cwIdTXLevel * 128);
    m_dmrTXLevel = q15_t(dmrTXLevel * 128);
    m_p25TXLevel = q15_t(p25TXLevel * 128);
    m_nxdnTXLevel =  q15_t(nxdnTXLevel * 128);

    ::LogInfoEx(LOG_SDR, "Modem %u (%s) RX LVL: %u CWID TX LVL: %u DMR TX LVL: %u P25 TX LVL: %u NXDN TX LVL: %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), m_rxLevel, m_cwIdTXLevel, m_dmrTXLevel, m_p25TXLevel, m_nxdnTXLevel);
    ::LogInfoEx(LOG_SDR, "Modem %u (%s) RX INVERT: %u TX INVERT: %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), m_rxInvert, m_txInvert);

    radio::RadioManager::instance().setChannelPolarity(m_modem->m_modemId, m_rxInvert, m_txInvert);
}

/* Sets the software Rx sample level. */

void IO::setRXLevel(uint8_t rxLevel)
{
    m_rxLevel = q15_t(rxLevel * 128);
}

/* Sets the RF parameters. */

uint8_t IO::setRFParams(uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower)
{
    m_rfPower = rfPower;

    // check frequency ranges
    if (!(
            /* 136 - 174 mhz */
            ((rxFreq >= VHF_MIN) && (rxFreq < VHF_MAX)) || ((txFreq >= VHF_MIN) && (txFreq < VHF_MAX)) ||
            /* 216 - 225 mhz */
            ((rxFreq >= VHF_220_MIN) && (rxFreq < VHF_220_MAX)) || ((txFreq >= VHF_220_MIN) && (txFreq < VHF_220_MAX)) ||
            /* 380 - 431 mhz */
            ((rxFreq >= UHF_380_MIN) && (rxFreq < UHF_380_MAX)) || ((txFreq >= UHF_380_MIN) && (txFreq < UHF_380_MAX)) ||
            /* 431 - 450 mhz */
            ((rxFreq >= UHF_1_MIN) && (rxFreq < UHF_1_MAX)) || ((txFreq >= UHF_1_MIN) && (txFreq < UHF_1_MAX)) ||
            /* 450 - 470 mhz */
            ((rxFreq >= UHF_2_MIN) && (rxFreq < UHF_2_MAX)) || ((txFreq >= UHF_2_MIN) && (txFreq < UHF_2_MAX)) ||
            /* 470 - 520 mhz */
            ((rxFreq >= UHF_T_MIN) && (rxFreq < UHF_T_MAX)) || ((txFreq >= UHF_T_MIN) && (txFreq < UHF_T_MAX)) ||
            /* 842 - 900 mhz */
            ((rxFreq >= UHF_800_MIN) && (rxFreq < UHF_800_MAX)) || ((txFreq >= UHF_800_MIN) && (txFreq < UHF_800_MAX)) ||
            /* 900 - 950 mhz */
            ((rxFreq >= UHF_900_MIN) && (rxFreq < UHF_900_MAX)) || ((txFreq >= UHF_900_MIN) && (txFreq < UHF_900_MAX))
        ))
        return RSN_INVALID_REQUEST;

    m_rxFrequency = rxFreq;
    m_txFrequency = txFreq;

    m_modem->setRFChannel(rxFreq, txFreq, rfPower, m_rxInvert, m_txInvert);

    ::LogInfoEx(LOG_SDR, "Modem %u (%s) RX FREQ: %u TX FREQ: %u PWR: %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), m_rxFrequency, m_txFrequency, m_rfPower);

    return RSN_OK;
}

/* Sets the RF adjustment parameters. */

void IO::setRFAdjust(int8_t dmrDiscBWAdj, int8_t p25DiscBWAdj, int8_t nxdnDiscBWAdj, int8_t dmrPostBWAdj, int8_t p25PostBWAdj, int8_t nxdnPostBWADJ)
{
    m_dmrDiscBWAdj = dmrDiscBWAdj;
    m_p25DiscBWAdj = p25DiscBWAdj;
    m_nxdnDiscBWAdj = nxdnDiscBWAdj;
    m_dmrPostBWAdj = dmrPostBWAdj;
    m_p25PostBWAdj = p25PostBWAdj;
    m_nxdnPostBWAdj = nxdnPostBWADJ;

    m_modem->writeDebug("IO::setRFAdjust() RF adjustment, discBW", dmrDiscBWAdj, p25DiscBWAdj, nxdnDiscBWAdj);
    m_modem->writeDebug("IO::setRFAdjust() RF adjustment, postBW", dmrPostBWAdj, p25PostBWAdj, nxdnPostBWADJ);
}

/* Sets the RF AFC parameters. */

void IO::setAFCParams(bool afcEnable, uint8_t afcKI, uint8_t afcKP, uint8_t afcRange)
{
    m_afcEnable = afcEnable;
    m_afcKI = afcKI;
    m_afcKP = afcKP;
    m_afcRange = afcRange;

    m_modem->writeDebug("IO::setAFCParams() AFC params", afcEnable, afcKI, afcKP, afcRange);
}

/* Helper to get the state of the ADC and DAC overflow flags. */

void IO::getOverflow(bool& adcOverflow, bool& dacOverflow)
{
    adcOverflow = m_adcOverflow > 0U;
    dacOverflow = m_dacOverflow > 0U;

    m_adcOverflow = 0U;
    m_dacOverflow = 0U;
}

/* Flag indicating the TX ring buffer has overflowed. */

bool IO::hasTXOverflow()
{
    return m_txBuffer.hasOverflowed();
}

/* Flag indicating the RX ring buffer has overflowed. */

bool IO::hasRXOverflow()
{
    return m_rxBuffer.hasOverflowed();
}

/* Flag indicating the air interface is locked out from transmitting. */

bool IO::hasLockout() const
{
    return m_lockout;
}

/* */

void IO::resetWatchdog()
{
    m_watchdog = 0U;
}

/* */

uint32_t IO::getWatchdog()
{
    return m_watchdog;
}

/* Gets the CPU type the firmware is running on. */

uint8_t IO::getCPU() const
{
    return CPU_TYPE_NATIVE_SDR;
}

/// <summary>
/// Gets the unique identifier for the air interface.
/// </summary>
/// <returns></returns>
void IO::getUDID(uint8_t* buffer)
{
    /* stub */
}

/* */

void IO::resetMCU()
{
    /* not supported for SDR devices */
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Starts hardware interrupts. */

void IO::startInt()
{
    ::LogInfoEx(LOG_SDR, "Modem %u (%s) host connected, starting Rx/Tx...", m_modem->m_modemId, m_modem->m_modemPty.c_str());

    /*
    ** bryanb: at this point we *should* have received configuration data from the host, which includes frequency
    **  we should create an entry point on the Modem class to call back and set the frequency data for this modem
    ** TODO TODO TODO
    */

    m_audioBufTx = std::vector<short>();
    m_audioBufRx = std::vector<short>();

    if (::pthread_mutex_init(&m_txLock, NULL) != 0) {
        ::LogError(LOG_SDR, "Tx thread lock failed?");
        ::LogFinalise();
        exit(-1);
    }

    if (::pthread_mutex_init(&m_rxLock, NULL) != 0) {
        ::LogError(LOG_SDR, "Rx thread lock failed?");
        ::LogFinalise();
        exit(-2);
    }

    ::pthread_create(&m_threadTx, NULL, txThreadHelper, this);
    ::pthread_create(&m_threadRx, NULL, rxThreadHelper, this);
    ::pthread_create(&m_threadStatus, NULL, modemStatusHelper, this);
}

/*  */

bool IO::getCOSInt()
{
    return m_cosInt;
}

/*  */

void IO::setPTTInt(bool on)
{
    m_ptt = on;
}

/*  */

void IO::setCOSInt(bool on)
{
    m_cosInt = on;
}

/*  */

void IO::setDMRInt(bool on)
{
    if (on != m_dmrMode)
        m_dmrModeToggle = true;

    m_dmrMode = on;
}

/*  */

void IO::setP25Int(bool on)
{
    if (on != m_p25Mode)
        m_p25ModeToggle = true;

    m_p25Mode = on;
}

/*  */

void IO::setNXDNInt(bool on)
{
    if (on != m_nxdnMode)
        m_nxdnModeToggle = true;

    m_nxdnMode = on;
}

/*  */

void IO::delayInt(unsigned int dly)
{
    ::usleep(dly * 1000U);
}

/*  */

void* IO::modemStatusHelper(void* arg)
{
    IO* io = (IO*)arg;
    if (io != nullptr) {
        while (!io->m_abort) {
            // log flag statuses
            if (io->m_cosPrev != io->m_cosInt) {
                ::LogInfoEx(LOG_SDR, "Modem %u (%s) COS %s", io->m_modem->m_modemId, io->m_modem->m_modemPty.c_str(), io->m_cosInt ? "DETECT" : "NO CARRIER");
                io->m_cosPrev = io->m_cosInt;
            }

            if (io->m_pttPrev != io->m_ptt) {
                ::LogInfoEx(LOG_SDR, "Modem %u (%s) PTT %s", io->m_modem->m_modemId, io->m_modem->m_modemPty.c_str(), io->m_ptt ? "TRANSMIT" : "IDLE");
                io->m_pttPrev = io->m_ptt;
            }

            if (io->m_dmrModeToggle) {
                ::LogInfoEx(LOG_SDR, "Modem %u (%s) DMR Mode %s", io->m_modem->m_modemId, io->m_modem->m_modemPty.c_str(), io->m_dmrMode ? "ENABLED" : "DISABLED");
                io->m_dmrModeToggle = false;
            }

            if (io->m_p25ModeToggle) {
                ::LogInfoEx(LOG_SDR, "Modem %u (%s) P25 Mode %s", io->m_modem->m_modemId, io->m_modem->m_modemPty.c_str(), io->m_p25Mode ? "ENABLED" : "DISABLED");
                io->m_p25ModeToggle = false;
            }

            if (io->m_nxdnModeToggle) {
                ::LogInfoEx(LOG_SDR, "Modem %u (%s) NXDN Mode %s", io->m_modem->m_modemId, io->m_modem->m_modemPty.c_str(), io->m_nxdnMode ? "ENABLED" : "DISABLED");
                io->m_nxdnModeToggle = false;
            }

            ::usleep(1000U);
        }
    }

    return nullptr;
}

/* Hardware interrupt handler. */

void IO::interrupt()
{
    int16_t sample = 0;
    uint8_t control = MARK_NONE;

    ::pthread_mutex_lock(&m_txLock);
    while (m_txBuffer.get(sample, control)) {
        // FM mode: stage real samples and emit fixed-size frames at modem cadence.
        m_audioBufTx.push_back(static_cast<short>(sample));

        if (m_audioBufTx.size() >= TX_FRAME_SAMPLES) {
            m_modem->transmitFMSamples(reinterpret_cast<const uint8_t*>(m_audioBufTx.data()),
                TX_FRAME_SAMPLES * sizeof(short));

            m_audioBufTx.erase(m_audioBufTx.begin(), m_audioBufTx.begin() + TX_FRAME_SAMPLES);
            ::pthread_mutex_unlock(&m_txLock);
            ::usleep(9600 * 3); // 9.6k baud for 30ms of samples to pace the modem correctly; TODO: replace with a more robust scheduler/timer mechanism
            ::pthread_mutex_lock(&m_txLock);
        }
    }
    ::pthread_mutex_unlock(&m_txLock);

    sample = 0;
    m_watchdog++;
}

/*  */

void* IO::txThreadHelper(void* arg)
{
    IO* p = (IO*)arg;

    while (!p->m_abort)
    {
        bool hasPendingTx = false;
        ::pthread_mutex_lock(&p->m_txLock);
        if (p->m_txBuffer.getData() > 0U) {
            hasPendingTx = true;
        } else {
            hasPendingTx = !p->m_audioBufTx.empty();
        }
        ::pthread_mutex_unlock(&p->m_txLock);

        if (!hasPendingTx) {
            // Keep refill latency below one scheduler millisecond to reduce FM queue starvation.
            ::usleep(250U);
            continue;
        }

        p->interrupt();
    }

    return NULL;
}

/*  */

void IO::noteRxRfActivity(uint32_t bytes, bool iqMode)
{
    const uint64_t now = monotonicMs();
    m_rxRfLastDataMs = now;
    m_rxRfBytes += bytes;
    m_rxRfBursts++;

    if (!m_rxRfActive) {
        m_rxRfActive = true;
        m_rxRfLastReportMs = now;
        ::LogDebugEx(LOG_SDR, "IO::interruptRx()", "Modem %u (%s) RX ACTIVE (%s)",
            m_modem->m_modemId, m_modem->m_modemPty.c_str(), iqMode ? "IQ" : "FM");
        return;
    }

    if ((now - m_rxRfLastReportMs) >= RX_RF_REPORT_INTERVAL_MS) {
        const unsigned long long elapsedMs = static_cast<unsigned long long>(now - m_rxRfLastReportMs);
        ::LogDebugEx(LOG_SDR, "IO::interruptRx()", "Modem %u (%s) RX ACTIVE (%s), %u bytes in %u bursts over %llums",
            m_modem->m_modemId, m_modem->m_modemPty.c_str(), iqMode ? "IQ" : "FM", m_rxRfBytes, m_rxRfBursts, elapsedMs);

        m_rxRfBytes = 0U;
        m_rxRfBursts = 0U;
        m_rxRfLastReportMs = now;
    }
}

/*  */

void IO::noteRxRfIdle()
{
    if (!m_rxRfActive)
        return;

    const uint64_t now = monotonicMs();
    if ((now - m_rxRfLastDataMs) < RX_RF_IDLE_INTERVAL_MS)
        return;

    ::LogDebugEx(LOG_SDR, "IO::interruptRx()", "Modem %u (%s) RX IDLE", m_modem->m_modemId, m_modem->m_modemPty.c_str());
    m_rxRfActive = false;
    m_rxRfBytes = 0U;
    m_rxRfBursts = 0U;
}

/*  */

void IO::interruptRx()
{
    uint8_t control = MARK_NONE;

    // FM mode: receive FM-demodulated real audio samples from the SDR path.
    uint8_t* samples = nullptr;
    int size = m_modem->readFMSamples(samples);
    if (size < 1 || samples == nullptr) {
        if (m_debug)
            noteRxRfIdle();
        return;
    }

    const uint8_t* sampleBytes = samples;

    if (m_debug)
        noteRxRfActivity(static_cast<uint32_t>(size), false);

    ::pthread_mutex_lock(&m_rxLock);

    for (int i = 0; i < size; i += 2) {
        int16_t sample = 0;
        ::memcpy(&sample, sampleBytes + i, sizeof(short));

        m_rxBuffer.put(sample, control);
        m_rssiBuffer.put(3U);
    }
    ::pthread_mutex_unlock(&m_rxLock);
}

/*  */

void* IO::rxThreadHelper(void* arg)
{
    IO* p = (IO*)arg;

    while (!p->m_abort)
        p->interruptRx();

    return NULL;
}
