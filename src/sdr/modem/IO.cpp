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

#define TX_HANG_MS 60U             // keep short burst continuity without long tail hold

#define RX_DEBUG_LOG_INTERVAL_MS 500U

#define TX_PROCESS_BLOCK_SAMPLES 240U      // match the RF runtime block size for lower latency

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
    m_txLastActivityMs(0U),
    m_abort(false),
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

    interruptRx();

    // use the COS line to lockout the modem
    if (m_modem->m_cosLockoutEnable) {
        m_lockout = getCOSInt();
    }

    bool hasPendingTxRing = false;
    ::pthread_mutex_lock(&m_txLock);
    hasPendingTxRing = (m_txBuffer.getData() > 0U);
    ::pthread_mutex_unlock(&m_txLock);

    const uint64_t nowMs = monotonicMs();
    if (m_modem->m_tx) {
        if (hasPendingTxRing) {
            interrupt();
        } else {
            const bool txHangExpired = (m_txLastActivityMs == 0U) || ((nowMs - m_txLastActivityMs) >= TX_HANG_MS);

            if (txHangExpired) {
                m_modem->m_tx = false;
                m_modem->setModemTxActive(false);

                if (m_debug)
                    ::LogDebugEx(LOG_SDR, "IO::process()", "no Tx data, TX OFF");
                setPTTInt(m_pttInvert ? true : false);
            }
        }
    } else {
        m_modem->setModemTxActive(false);
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
        m_txLastActivityMs = monotonicMs();

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

    // TX ring is consumed by the modem clock thread; writes must be serialized
    // with reads to prevent ring index corruption under load.
    ::pthread_mutex_lock(&m_txLock);
    for (uint16_t i = 0U; i < length; i++) {
        q31_t res1 = samples[i] * txLevel;
        q15_t res2 = q15_t(__SSAT((res1 >> 15), 16));

        // Detect DAC overflow.
        if (res2 >= 4095 || res2 <= -4095)
            m_dacOverflow++;

        if (control == NULL)
            m_txBuffer.put(res2, MARK_NONE);
        else
            m_txBuffer.put(res2, control[i]);
    }
    ::pthread_mutex_unlock(&m_txLock);

    m_txLastActivityMs = monotonicMs();
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
    m_txLastActivityMs = monotonicMs();

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

    ::LogInfoEx(LOG_SDR, "Modem %u (%s) rxLevel = %u, cwIdTXLevel = %u, dmrTXLevel = %u, p25TXLevel = %u, nxdnTXLevel = %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), m_rxLevel, m_cwIdTXLevel, m_dmrTXLevel, m_p25TXLevel, m_nxdnTXLevel);
    ::LogInfoEx(LOG_SDR, "Modem %u (%s) rxInvert = %u, txInvert = %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), m_rxInvert, m_txInvert);

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

    ::LogInfoEx(LOG_SDR, "Modem %u (%s) rxFrequency = %u, txFrequency = %u, rfPower = %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), m_rxFrequency, m_txFrequency, m_rfPower);

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

    radio::RadioManager::instance().setChannelAFC(m_modem->m_modemId, afcEnable, afcKI, afcKP, afcRange);

    ::LogInfoEx(LOG_SDR, "Modem %u (%s) AFC params, enable = %u, KI = %u, KP = %u, range = %u", m_modem->m_modemId, m_modem->m_modemPty.c_str(), afcEnable, afcKI, afcKP, afcRange);
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

void IO::interrupt()
{
    int16_t sample = 0;
    uint8_t control = MARK_NONE;
    int16_t txSamples[TX_PROCESS_BLOCK_SAMPLES];
    uint8_t txControls[TX_PROCESS_BLOCK_SAMPLES];
    size_t txCount = 0U;

    ::pthread_mutex_lock(&m_txLock);
    while (m_txBuffer.get(sample, control)) {
        txSamples[txCount] = sample;
        txControls[txCount] = control;
        txCount++;

        if (txCount >= TX_PROCESS_BLOCK_SAMPLES) {
            m_modem->transmitRFSamples(txSamples, txControls, txCount);
            txCount = 0U;
        }
    }

    if (txCount > 0U)
        m_modem->transmitRFSamples(txSamples, txControls, txCount);

    ::pthread_mutex_unlock(&m_txLock);

    m_watchdog++;
}

void IO::logRxRfSamples(uint32_t bytes, bool iqMode)
{
    const uint64_t now = monotonicMs();
    m_rxRfLastDataMs = now;
    m_rxRfBytes += bytes;
    m_rxRfBursts++;

    if ((now - m_rxRfLastReportMs) >= RX_RF_REPORT_INTERVAL_MS) {
        const unsigned long long elapsedMs = static_cast<unsigned long long>(now - m_rxRfLastReportMs);
        ::LogDebugEx(LOG_SDR, "IO::interruptRx()", "Modem %u (%s) RX SAMPLES (%s), %u bytes in %u bursts over %llums",
            m_modem->m_modemId, m_modem->m_modemPty.c_str(), iqMode ? "IQ" : "FM", m_rxRfBytes, m_rxRfBursts, elapsedMs);

        m_rxRfBytes = 0U;
        m_rxRfBursts = 0U;
        m_rxRfLastReportMs = now;
    }
}

/*  */

void IO::interruptRx()
{
    static uint64_t lastRxDebugLogMs = 0U;

    for (;;) {
        int16_t* samples = nullptr;
        uint8_t* controls = nullptr;
        uint16_t* rssi = nullptr;
        const int count = m_modem->readRFSamples(samples, controls, rssi);
        if (count < 1 || samples == nullptr)
            break;

        if (m_debug)
            logRxRfSamples(static_cast<uint32_t>(count * sizeof(int16_t)), true);

        for (int base = 0; base < count; base += RX_BLOCK_SIZE) {
            q15_t blockSamples[RX_BLOCK_SIZE] = {0};
            uint8_t blockControl[RX_BLOCK_SIZE] = {MARK_NONE, MARK_NONE};
            uint16_t blockRssi[RX_BLOCK_SIZE] = {3U, 3U};

            const int blockCount = std::min<int>(RX_BLOCK_SIZE, count - base);
            for (int i = 0; i < blockCount; i++) {
                const int idx = base + i;
                const int16_t sample = samples[idx];
                const uint8_t sampleControl = (controls != nullptr) ? controls[idx] : MARK_NONE;
                const uint16_t sampleRssi = (rssi != nullptr) ? rssi[idx] : 3U;

                // Detect ADC overflow.
                if (m_detect && (sample == 0U || sample == 4095U))
                    m_adcOverflow++;

                q31_t res1 = sample * m_rxLevel;
                blockSamples[i] = q15_t(__SSAT((res1 >> 15), 16));
                blockControl[i] = sampleControl;
                blockRssi[i] = sampleRssi;

                if (m_debug && sampleRssi > 1024) {
                    const uint64_t nowMs = monotonicMs();
                    if (lastRxDebugLogMs == 0U || (nowMs - lastRxDebugLogMs) >= RX_DEBUG_LOG_INTERVAL_MS) {
                        lastRxDebugLogMs = nowMs;
                        ::LogDebugEx(LOG_SDR, "IO::interruptRx()", "Modem %u (%s) RX SAMPLE, sample = %d, scaled = %d, control = %u, rssi = %u, rxLvlQ15 = %d",
                            m_modem->m_modemId, m_modem->m_modemPty.c_str(), sample, blockSamples[i], sampleControl, sampleRssi, m_rxLevel);
                    }
                }
            }

            if (m_lockout)
                return;

            q15_t dcSamples[RX_BLOCK_SIZE];
            q15_t* activeSamples = blockSamples;
            if (m_modem->m_dcBlockerEnable) {
                q31_t q31Samples[RX_BLOCK_SIZE];

                ::arm_q15_to_q31(blockSamples, q31Samples, RX_BLOCK_SIZE);

                q31_t dcValues[RX_BLOCK_SIZE];
                ::arm_biquad_cascade_df1_q31(&m_dcFilter, q31Samples, dcValues, RX_BLOCK_SIZE);

                q31_t dcLevel = 0;
                for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
                    dcLevel += dcValues[i];
                dcLevel /= RX_BLOCK_SIZE;

                q15_t offset = q15_t(__SSAT((dcLevel >> 16), 16));

                for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
                    dcSamples[i] = blockSamples[i] - offset;

                activeSamples = dcSamples;
            }

            /** Idle Modem State */
            if (m_modem->m_modemState == STATE_IDLE) {
                /** Project 25 */
                if (m_modem->m_p25Enable) {
                    q15_t c4fmSamples[RX_BLOCK_SIZE];
                    ::arm_fir_fast_q15(&m_boxcar_5_Filter, activeSamples, c4fmSamples, RX_BLOCK_SIZE);
                    m_modem->m_p25RX.samples(c4fmSamples, blockRssi, RX_BLOCK_SIZE);
                }

                /** Digital Mobile Radio */
                if (m_modem->m_dmrEnable) {
                    q15_t c4fmSamples[RX_BLOCK_SIZE];
                    ::arm_fir_fast_q15(&m_rrc_0_2_Filter, blockSamples, c4fmSamples, RX_BLOCK_SIZE);

                    if (m_modem->m_duplex)
                        m_modem->m_dmrIdleRX.samples(c4fmSamples, RX_BLOCK_SIZE);
                    else
                        m_modem->m_dmrDMORX.samples(c4fmSamples, blockRssi, RX_BLOCK_SIZE);
                }

                /** Next Generation Digital Narrowband */
                if (m_modem->m_nxdnEnable) {
                    q15_t c4fmSamples[RX_BLOCK_SIZE];
#if NXDN_BOXCAR_FILTER
                    ::arm_fir_fast_q15(&m_boxcar_10_Filter, activeSamples, c4fmSamples, RX_BLOCK_SIZE);
#else
                    q15_t c4fmRCSamples[RX_BLOCK_SIZE];
                    ::arm_fir_fast_q15(&m_nxdn_0_2_Filter, activeSamples, c4fmRCSamples, RX_BLOCK_SIZE);
                    ::arm_fir_fast_q15(&m_nxdn_ISinc_Filter, c4fmRCSamples, c4fmSamples, RX_BLOCK_SIZE);
#endif
                    m_modem->m_nxdnRX.samples(c4fmSamples, blockRssi, RX_BLOCK_SIZE);
                }
            }
            else if (m_modem->m_modemState == STATE_DMR) {
                /** Digital Mobile Radio */
                if (m_modem->m_dmrEnable) {
                    q15_t c4fmSamples[RX_BLOCK_SIZE];
                    ::arm_fir_fast_q15(&m_rrc_0_2_Filter, blockSamples, c4fmSamples, RX_BLOCK_SIZE);

                    if (m_modem->m_duplex) {
                        if (m_modem->m_tx)
                            m_modem->m_dmrRX.samples(c4fmSamples, blockRssi, blockControl, RX_BLOCK_SIZE);
                        else
                            m_modem->m_dmrIdleRX.samples(c4fmSamples, RX_BLOCK_SIZE);
                    }
                    else {
                        m_modem->m_dmrDMORX.samples(c4fmSamples, blockRssi, RX_BLOCK_SIZE);
                    }
                }
            }
            else if (m_modem->m_modemState == STATE_P25) {
                /** Project 25 */
                if (m_modem->m_p25Enable) {
                    q15_t c4fmSamples[RX_BLOCK_SIZE];
                    ::arm_fir_fast_q15(&m_boxcar_5_Filter, activeSamples, c4fmSamples, RX_BLOCK_SIZE);
                    m_modem->m_p25RX.samples(c4fmSamples, blockRssi, RX_BLOCK_SIZE);
                }
            }
            else if (m_modem->m_modemState == STATE_NXDN) {
                /** Next Generation Digital Narrowband */
                if (m_modem->m_nxdnEnable) {
                    q15_t c4fmSamples[RX_BLOCK_SIZE];
#if NXDN_BOXCAR_FILTER
                    ::arm_fir_fast_q15(&m_boxcar_10_Filter, activeSamples, c4fmSamples, RX_BLOCK_SIZE);
#else
                    q15_t c4fmRCSamples[RX_BLOCK_SIZE];
                    ::arm_fir_fast_q15(&m_nxdn_0_2_Filter, activeSamples, c4fmRCSamples, RX_BLOCK_SIZE);
                    ::arm_fir_fast_q15(&m_nxdn_ISinc_Filter, c4fmRCSamples, c4fmSamples, RX_BLOCK_SIZE);
#endif
                    m_modem->m_nxdnRX.samples(c4fmSamples, blockRssi, RX_BLOCK_SIZE);
                }
            }
            else if (m_modem->m_modemState == STATE_RSSI_CAL) {
                m_modem->m_calRSSI.samples(blockRssi, RX_BLOCK_SIZE);
            }
        }
    }
}

