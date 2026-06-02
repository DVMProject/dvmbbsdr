// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 */
#include "radio/RadioManager.h"

#include "common/Log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unistd.h>

#if defined(HAS_SOAPYSDR)
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <SoapySDR/Errors.h>
#endif

using namespace radio;

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

static constexpr double MODEM_SAMPLE_RATE = 24000.0;

static constexpr uint32_t BLOCK_MS = 10U;

static constexpr size_t PROCESS_BLOCK_SAMPLES = 240U;

static constexpr size_t MAX_TX_QUEUE_SAMPLES = 2400U;
static constexpr size_t MAX_RX_QUEUE_SAMPLES = 2400U;

static constexpr size_t MAX_READ_SAMPLES = PROCESS_BLOCK_SAMPLES;

static constexpr size_t DEFAULT_CONTROL_DELAY_SAMPLES = 96U;
static constexpr size_t LATENCY_BLOCKS = 3U;

static constexpr size_t FDUDC_FILTER_LEN = 11U;
static constexpr uint32_t MAX_TX_TIMEOUT_RETRIES = 8U;
static constexpr uint64_t DEVICE_DIAG_INTERVAL_MS = 1000U;
static constexpr long SOAPY_STREAM_TIMEOUT_US = 20000L;

static constexpr double NOMINAL_CARRIER_BW_HZ = 12500.0;
static constexpr double MIN_GUARD_BAND_HZ = 2500.0;

static constexpr int32_t FM_DEVIATION = 550000;
static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWO_PI_F = 6.28318530717958647692f;

// ---------------------------------------------------------------------------
//  Global Functions
// ---------------------------------------------------------------------------

/**
 * @brief Wraps a phase value to the range of -pi to pi. This function is used to ensure that phase values remain 
 * within a standard range for processing, particularly in the context of NCO phase accumulation and modulation. By 
 * wrapping the phase, we can avoid issues with phase overflow and maintain consistent behavior in the signal 
 * processing algorithms.
 * @param phase The input phase value to wrap, in radians.
 * @returns float The wrapped phase value, constrained to the range of -pi to pi.
 */
static inline float wrapPhase(float phase)
{
    while (phase > PI_F)
        phase -= TWO_PI_F;
    while (phase < -PI_F)
        phase += TWO_PI_F;
    return phase;
}

/**
 * @brief Clamps the RSSI value derived from a complex IQ sample to the range of 0 to 65535. This function calculates
 * the power level of the IQ sample, scales it to fit within the 16-bit unsigned integer range, and ensures that it 
 * does not exceed the maximum representable value. This is important for accurately representing signal strength in a 
 * format that can be easily used for diagnostics and processing within the RadioManager.
 * @param iq The complex IQ sample from which to derive the RSSI value.
 * @returns uint16_t The clamped RSSI value corresponding to the power level of the IQ sample, scaled to fit within the 
 *  range of 0 to 65535.
 */
static inline uint16_t clampRssiFromIq(const std::complex<float>& iq)
{
    const float level = 100000000.0f * std::norm(iq);
    if (level <= 0.0f)
        return 0U;
    if (level >= 65535.0f)
        return 65535U;
    return static_cast<uint16_t>(level + 0.5f);
}

/**
 * @brief Clamps a floating-point value to the range of -32768 to 32767, suitable for Q15 representation.
 * This function ensures that the input value does not exceed the limits of a 16-bit signed integer, which is
 * important for maintaining signal integrity when converting floating-point samples to fixed-point format.
 * @param v The input floating-point value to clamp.
 * @returns int16_t The clamped value, constrained to the range of -32768 to 32767.
 */
static inline int16_t clampQ15(float v)
{
    if (v > 32767.0f)
        return 32767;
    if (v < -32768.0f)
        return -32768;
    return static_cast<int16_t>(std::lround(v));
}

// ---------------------------------------------------------------------------
//  Externs
// ---------------------------------------------------------------------------

extern uint64_t monotonicMs();

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Gets the singleton instance of the RadioManager. */

RadioManager& RadioManager::instance()
{
    static RadioManager s_instance;
    return s_instance;
}

/* Initializes the RadioManager with the given configuration and debug flag. */

bool RadioManager::initialize(yaml::Node& conf, bool debug)
{
    shutdown();

    m_debug = debug;

#if defined(HAS_SOAPYSDR)
    SoapySDR::setLogLevel(debug ? SOAPY_SDR_INFO : SOAPY_SDR_WARNING);
#endif

    // scope is intentional
    {
        std::lock_guard<std::mutex> lock(m_stateLock);
        parseConfig(conf);
    }

#if defined(HAS_SOAPYSDR)
    // start all devices after parsing config to avoid issues with setting parameters before device is initialized
    for (size_t i = 0U; i < m_devices.size(); i++)
        startSoapyDevice(i);
#endif

    m_running = true;
    m_runtimeThread = std::thread(&RadioManager::runtimeLoop, this);

    ::LogInfoEx(LOG_SDR, "RadioManager initialized (single-carrier FM IQ path)");
    return true;
}

/* Shuts down the RadioManager, stopping all devices and clearing all channels. */

void RadioManager::shutdown()
{
    m_running = false;

    if (m_runtimeThread.joinable())
        m_runtimeThread.join();

#if defined(HAS_SOAPYSDR)
    stopAllSoapyDevices();
#endif

    std::lock_guard<std::mutex> lock(m_stateLock);
    m_channels.clear();
    m_modemRxDevice.clear();
    m_modemTxDevice.clear();
    m_devices.clear();
}

/* Helper to set the RF channel polarity for a specific modem channel. */

void RadioManager::setChannelPolarity(uint8_t modemId, bool rxInvert, bool txInvert)
{
    ChannelState* channel = getChannel(modemId);
    if (channel == nullptr)
        return;

    std::lock_guard<std::mutex> lock(channel->lock);
    channel->rxInvert = rxInvert;
    channel->txInvert = txInvert;
}

/* Helper to set the RF channel parameters for a specific modem channel. */

void RadioManager::setChannelParams(uint8_t modemId, uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower, bool rxInvert, bool txInvert)
{
    std::lock_guard<std::mutex> lock(m_stateLock);

    if (m_primaryModemValid && modemId != m_primaryModemId)
        return;

    if (!m_primaryModemValid) {
        m_primaryModemId = modemId;
        m_primaryModemValid = true;
    }

    ChannelState& channel = m_channels[m_primaryModemId];
    if (channel.modemId == 0U) {
        channel.modemId = m_primaryModemId;
        channel.rxDevice = 0U;
        channel.txDevice = 0U;
        channel.txActive = false;
        channel.afcEnabled = false;
        channel.afcKI = 0U;
        channel.afcKP = 0U;
        channel.afcRange = 0U;
        channel.rxOffsetHz = 0.0;
        channel.txOffsetHz = 0.0;
        channel.txPhaseWord = 0;
        channel.txNcoPhase = 0.0f;
        channel.txNcoStep = 0.0f;
        channel.prevRx = std::complex<float>(0.0f, 0.0f);
        channel.rxNcoPhase = 0.0f;
        channel.rxNcoStep = 0.0f;
        channel.fdudc = nullptr;
        channel.fdudcNum = 0U;
        channel.fdudcDen = 0U;
        channel.controlDelaySamples = DEFAULT_CONTROL_DELAY_SAMPLES;
        channel.txHoldSample = 0;
        channel.txHoldControl = 0U;
        channel.txInterpCounter = 0U;
        channel.rxDecimCounter = 0U;
        channel.rxDecimAcc = 0.0f;
        channel.rxRssiAcc = 0.0f;
        channel.rxRssiCount = 0U;
        channel.delayedControl.assign(channel.controlDelaySamples, 0U);

        auto rxIt = m_modemRxDevice.find(m_primaryModemId);
        auto txIt = m_modemTxDevice.find(m_primaryModemId);
        if (rxIt != m_modemRxDevice.end())
            channel.rxDevice = rxIt->second;
        if (txIt != m_modemTxDevice.end())
            channel.txDevice = txIt->second;
    }

    channel.rxFreq = rxFreq;
    channel.txFreq = txFreq;
    channel.rfPower = rfPower;
    channel.rxInvert = rxInvert;
    channel.txInvert = txInvert;

    recomputeDeviceCenters();
    updateChannelOffsets();
    updateChannelSpacingMetrics();

#if defined(HAS_SOAPYSDR)
    if (channel.rxDevice < m_devices.size()) {
        DeviceState& rxDev = m_devices[channel.rxDevice];
        if (rxDev.soapyDevice != nullptr)
            rxDev.soapyDevice->setFrequency(SOAPY_SDR_RX, 0U, rxDev.rxCenterHz);
    }

    if (channel.txDevice < m_devices.size()) {
        DeviceState& txDev = m_devices[channel.txDevice];
        if (txDev.soapyDevice != nullptr)
            txDev.soapyDevice->setFrequency(SOAPY_SDR_TX, 0U, txDev.txCenterHz);
    }
#endif
}

/* Helper to set the AFC parameters for a specific modem channel. */

void RadioManager::setChannelAFC(uint8_t modemId, bool enabled, uint8_t ki, uint8_t kp, uint8_t range)
{
    ChannelState* channel = getChannel(modemId);
    if (channel == nullptr)
        return;

    std::lock_guard<std::mutex> lock(channel->lock);
    channel->afcEnabled = enabled;
    channel->afcKI = ki;
    channel->afcKP = kp;
    channel->afcRange = range;
}

/* Helper to set the TX active state for a specific modem channel. */

void RadioManager::setChannelTxActive(uint8_t modemId, bool active)
{
    ChannelState* channel = getChannel(modemId);
    if (channel == nullptr)
        return;

    std::lock_guard<std::mutex> lock(channel->lock);
    channel->txActive = active;
}

/* Writes samples to the TX queue of a specific modem channel. */

size_t RadioManager::writeChannelTxSamples(uint8_t modemId, const int16_t* samples, const uint8_t* control, size_t sampleCount)
{
    if (samples == nullptr || sampleCount == 0U)
        return 0U;

    ChannelState* channel = getChannel(modemId);
    if (channel == nullptr)
        return 0U;

    std::lock_guard<std::mutex> lock(channel->lock);

    // queue samples until we reach the maximum allowed in the TX queue, if the queue is full, we stop queuing 
    // additional samples to avoid unbounded memory growth
    size_t queued = 0U;
    for (size_t i = 0U; i < sampleCount; i++) {
        if (channel->txSamples.size() >= MAX_TX_QUEUE_SAMPLES)
            break;

        channel->txSamples.push_back(samples[i]);
        channel->txControl.push_back(control != nullptr ? control[i] : 0U);
        queued++;
    }

    return queued;
}

/* Reads samples from the RX queue of a specific modem channel. */

int RadioManager::readChannelRxSamples(uint8_t modemId, int16_t*& samples, uint8_t*& control, uint16_t*& rssi)
{
    samples = nullptr;
    control = nullptr;
    rssi = nullptr;

    ChannelState* channel = getChannel(modemId);
    if (channel == nullptr)
        return 0;

    std::lock_guard<std::mutex> lock(channel->lock);
    if (channel->rxSamples.empty())
        return 0;

    const size_t count = std::min(channel->rxSamples.size(), MAX_READ_SAMPLES);

    channel->readSamples.resize(count);
    channel->readControl.resize(count);
    channel->readRssi.resize(count);

    // copy samples from the RX queue to the output buffers, and remove them from the RX queue
    for (size_t i = 0U; i < count; i++) {
        channel->readSamples[i] = channel->rxSamples.front();
        channel->readControl[i] = channel->rxControl.front();
        channel->readRssi[i] = channel->rxRssi.front();

        channel->rxSamples.pop_front();
        channel->rxControl.pop_front();
        channel->rxRssi.pop_front();
    }

    samples = channel->readSamples.data();
    control = channel->readControl.data();
    rssi = channel->readRssi.data();
    return static_cast<int>(count);
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the RadioManager class. */

RadioManager::RadioManager() :
    m_running(false),
    m_debug(false),
    m_runtimeThread(),
    m_stateLock(),
    m_devices(),
    m_modemRxDevice(),
    m_modemTxDevice(),
    m_channels(),
    m_primaryModemId(0U),
    m_primaryModemValid(false)
{
    /* stub */
}

/* Finalizes an instance of the RadioManager class. */

RadioManager::~RadioManager()
{
    shutdown();
}

/* Parses the YAML configuration to set up the SDR devices and channels. */

void RadioManager::parseConfig(yaml::Node& conf)
{
    m_devices.clear();
    m_modemRxDevice.clear();
    m_modemTxDevice.clear();
    m_primaryModemId = 0U;
    m_primaryModemValid = false;

    yaml::Node sdrConf = conf["sdr"];
    yaml::Node defaults = sdrConf["defaults"];

    DeviceState defaultDev = makeDefaultDevice();
    defaultDev.sampleRate = defaults["sampleRate"].as<double>(defaultDev.sampleRate);
    defaultDev.rxGain = defaults["rxGain"].as<double>(defaultDev.rxGain);
    defaultDev.txGain = defaults["txGain"].as<double>(defaultDev.txGain);
    defaultDev.rxBandwidth = defaults["rxBandwidth"].as<double>(defaultDev.rxBandwidth);
    defaultDev.txBandwidth = defaults["txBandwidth"].as<double>(defaultDev.txBandwidth);
    defaultDev.freqCorrPpm = defaults["freqCorrPpm"].as<double>(defaultDev.freqCorrPpm);
    defaultDev.rxCenterOffsetHz = defaults["rxCenterOffsetHz"].as<double>(defaultDev.rxCenterOffsetHz);
    defaultDev.txCenterOffsetHz = defaults["txCenterOffsetHz"].as<double>(defaultDev.txCenterOffsetHz);
    defaultDev.rxAntenna = defaults["rxAntenna"].as<std::string>(defaultDev.rxAntenna);
    defaultDev.txAntenna = defaults["txAntenna"].as<std::string>(defaultDev.txAntenna);
    defaultDev.clockSource = defaults["clockSource"].as<std::string>(defaultDev.clockSource);
    defaultDev.timeSource = defaults["timeSource"].as<std::string>(defaultDev.timeSource);
    defaultDev.rxGainElement = defaults["rxGainElement"].as<std::string>(defaultDev.rxGainElement);
    defaultDev.txGainElement = defaults["txGainElement"].as<std::string>(defaultDev.txGainElement);

    // only a single SDR device is supported in the strict 1:1 runtime.
    yaml::Node devicesNode = sdrConf["devices"];
    if (devicesNode.size() == 0U) {
        m_devices.push_back(defaultDev);
    } else {
        if (devicesNode.size() > 1U) {
            ::LogError(LOG_SDR, "Only one SDR device configuration is supported; using devices[0]");
        }

        yaml::Node& dev = devicesNode[0U];
        DeviceState state = defaultDev;
        state.args = dev["args"].as<std::string>(state.args);
        state.sampleRate = dev["sampleRate"].as<double>(state.sampleRate);
        state.rxGain = dev["rxGain"].as<double>(state.rxGain);
        state.txGain = dev["txGain"].as<double>(state.txGain);
        state.rxBandwidth = dev["rxBandwidth"].as<double>(state.rxBandwidth);
        state.txBandwidth = dev["txBandwidth"].as<double>(state.txBandwidth);
        state.freqCorrPpm = dev["freqCorrPpm"].as<double>(state.freqCorrPpm);
        state.rxCenterOffsetHz = dev["rxCenterOffsetHz"].as<double>(state.rxCenterOffsetHz);
        state.txCenterOffsetHz = dev["txCenterOffsetHz"].as<double>(state.txCenterOffsetHz);
        state.rxAntenna = dev["rxAntenna"].as<std::string>(state.rxAntenna);
        state.txAntenna = dev["txAntenna"].as<std::string>(state.txAntenna);
        state.clockSource = dev["clockSource"].as<std::string>(state.clockSource);
        state.timeSource = dev["timeSource"].as<std::string>(state.timeSource);
        state.rxGainElement = dev["rxGainElement"].as<std::string>(state.rxGainElement);
        state.txGainElement = dev["txGainElement"].as<std::string>(state.txGainElement);

        m_devices.push_back(state);
    }

    if (m_devices.empty())
        m_devices.push_back(defaultDev);

    // only one modem is supported and it is bound 1:1 to device 0.
    yaml::Node modems = conf["modems"];
    if (modems.size() == 0U) {
        const uint8_t modemId = 1U;
        m_modemRxDevice[modemId] = 0U;
        m_modemTxDevice[modemId] = 0U;
        m_primaryModemId = modemId;
        m_primaryModemValid = true;
    } else {
        if (modems.size() > 1U) {
            ::LogError(LOG_SDR, "Only one modem configuration is supported; using modems[0]");
        }

        const size_t i = 0U;
        yaml::Node modemNode = modems[i];
        yaml::Node radioNode = modemNode["radio"];

        size_t rxDevice = radioNode["rxDevice"].as<size_t>(std::numeric_limits<size_t>::max());
        size_t txDevice = radioNode["txDevice"].as<size_t>(std::numeric_limits<size_t>::max());
        size_t bothDevice = radioNode["device"].as<size_t>(0U);

        if (rxDevice == std::numeric_limits<size_t>::max())
            rxDevice = bothDevice;
        if (txDevice == std::numeric_limits<size_t>::max())
            txDevice = bothDevice;

        if (rxDevice >= m_devices.size())
            rxDevice = 0U;
        if (txDevice >= m_devices.size())
            txDevice = 0U;

        const uint8_t modemId = static_cast<uint8_t>(i + 1U);
        m_modemRxDevice[modemId] = rxDevice;
        m_modemTxDevice[modemId] = txDevice;
        m_primaryModemId = modemId;
        m_primaryModemValid = true;
    }

    recomputeDeviceCenters();
    updateChannelOffsets();
    updateChannelSpacingMetrics();
}

/* Recomputes the center frequencies for each SDR device based on the assigned channels and their frequencies. */

void RadioManager::recomputeDeviceCenters()
{
    if (m_devices.empty())
        return;

    std::vector<bool> rxHave(m_devices.size(), false);
    std::vector<bool> txHave(m_devices.size(), false);
    std::vector<uint32_t> rxMin(m_devices.size(), std::numeric_limits<uint32_t>::max());
    std::vector<uint32_t> rxMax(m_devices.size(), 0U);
    std::vector<uint32_t> txMin(m_devices.size(), std::numeric_limits<uint32_t>::max());
    std::vector<uint32_t> txMax(m_devices.size(), 0U);

    // iterate through channels to find min/max RX and TX frequencies for each device, which we use to compute the center
    // frequencies for each device based on the assigned channels. This allows us to support dynamic retuning of channels 
    // without needing to restart or reinitialize the devices, as the center frequencies will be automatically
    for (auto& kv : m_channels) {
        ChannelState& channel = kv.second;
        const size_t rxDev = std::min(channel.rxDevice, m_devices.size() - 1U);
        const size_t txDev = std::min(channel.txDevice, m_devices.size() - 1U);

        if (channel.rxFreq > 0U) {
            rxHave[rxDev] = true;
            rxMin[rxDev] = std::min(rxMin[rxDev], channel.rxFreq);
            rxMax[rxDev] = std::max(rxMax[rxDev], channel.rxFreq);
        }

        if (channel.txFreq > 0U) {
            txHave[txDev] = true;
            txMin[txDev] = std::min(txMin[txDev], channel.txFreq);
            txMax[txDev] = std::max(txMax[txDev], channel.txFreq);
        }
    }

    // compute center frequencies for each device based on the min/max frequencies of the assigned channels, with an 
    // optional offset from the config
    for (size_t i = 0U; i < m_devices.size(); i++) {
        DeviceState& dev = m_devices[i];
        dev.rxCenterHz = rxHave[i] ? (0.5 * static_cast<double>(rxMin[i] + rxMax[i]) + dev.rxCenterOffsetHz) : dev.rxCenterOffsetHz;
        dev.txCenterHz = txHave[i] ? (0.5 * static_cast<double>(txMin[i] + txMax[i]) + dev.txCenterOffsetHz) : dev.txCenterOffsetHz;

        if (m_debug) {
            ::LogDebugEx(LOG_SDR, "RadioManager::recomputeDeviceCenters()", "dev=%zu sampleRate=%.0f rxCenter=%.1f txCenter=%.1f",
                i, dev.sampleRate, dev.rxCenterHz, dev.txCenterHz);
        }
    }
}

/* Updates the channel frequency offsets based on the current device center frequencies. */

void RadioManager::updateChannelOffsets()
{
    if (m_devices.empty())
        return;

    // iterate through channels to compute the RX and TX frequency offsets from the device center frequencies, which we 
    // use to compute the NCO steps for each channel. This allows us to support dynamic retuning of channels without
    // needing to restart or reinitialize the devices, as the offsets and NCO steps will be automatically updated based 
    // on the new center frequencies
    for (auto& kv : m_channels) {
        ChannelState& channel = kv.second;

        const size_t rxDev = std::min(channel.rxDevice, m_devices.size() - 1U);
        const size_t txDev = std::min(channel.txDevice, m_devices.size() - 1U);

        const DeviceState& rxDevice = m_devices[rxDev];
        const DeviceState& txDevice = m_devices[txDev];

        channel.rxOffsetHz = static_cast<double>(channel.rxFreq) - rxDevice.rxCenterHz;
        channel.txOffsetHz = static_cast<double>(channel.txFreq) - txDevice.txCenterHz;

        const double srRx = std::max(1.0, rxDevice.sampleRate);
        const double srTx = std::max(1.0, txDevice.sampleRate);

        channel.rxNcoStep = static_cast<float>(TWO_PI_F * channel.rxOffsetHz / srRx);
        channel.txNcoStep = static_cast<float>(TWO_PI_F * channel.txOffsetHz / srTx);

        const uint32_t rxDecim = static_cast<uint32_t>(std::max<long long>(1LL, std::llround(srRx / MODEM_SAMPLE_RATE)));
        const uint32_t txInterp = static_cast<uint32_t>(std::max<long long>(1LL, std::llround(srTx / MODEM_SAMPLE_RATE)));

        m_devices[rxDev].hwToModemDecim = rxDecim;
        m_devices[txDev].modemToHwInterp = txInterp;
    }
}

/* Updates the channel spacing metrics for diagnostics and monitoring. */

void RadioManager::updateChannelSpacingMetrics()
{
    if (m_devices.empty())
        return;

    // iterate through channels to group the TX frequencies by device, then compute the minimum carrier spacing and occupied
    // bandwidth for each device based on the assigned channels. This allows us to monitor the channel spacing and bandwidth 
    // usage for each device, and log warnings if the spacing violates the recommended guard band requirements
    std::vector<std::vector<double>> deviceFreqs(m_devices.size());
    for (const auto& kv : m_channels) {
        const ChannelState& channel = kv.second;
        if (channel.txFreq == 0U)
            continue;

        const size_t dev = std::min(channel.txDevice, m_devices.size() - 1U);
        deviceFreqs[dev].push_back(static_cast<double>(channel.txFreq));
    }

    // compute the minimum carrier spacing and occupied bandwidth for each device based on the assigned channel frequencies, and log
    // warnings if the spacing violates the recommended guard band requirements. The minimum carrier spacing is computed as 
    // the smallest difference between adjacent channel frequencies, and the occupied bandwidth is computed as the difference 
    // between the maximum and minimum channel frequencies
    for (size_t dev = 0U; dev < m_devices.size(); dev++) {
        auto& freqs = deviceFreqs[dev];
        std::sort(freqs.begin(), freqs.end());

        m_devices[dev].minCarrierSpacingHz = 0.0;
        m_devices[dev].occupiedBandwidthHz = 0.0;
        m_devices[dev].guardBandViolated = false;

        if (freqs.empty())
            continue;

        m_devices[dev].occupiedBandwidthHz = (freqs.back() - freqs.front()) + NOMINAL_CARRIER_BW_HZ;

        if (freqs.size() > 1U) {
            double minSpacing = std::numeric_limits<double>::max();
            for (size_t i = 1U; i < freqs.size(); i++) {
                const double spacing = freqs[i] - freqs[i - 1U];
                minSpacing = std::min(minSpacing, spacing);
            }

            m_devices[dev].minCarrierSpacingHz = minSpacing;
            m_devices[dev].guardBandViolated = (minSpacing < (NOMINAL_CARRIER_BW_HZ + MIN_GUARD_BAND_HZ));

            if (m_devices[dev].guardBandViolated) {
                ::LogWarning(LOG_SDR, "Device %zu carrier spacing %.1f Hz is below recommended %.1f Hz (channel %.1f + guard %.1f)",
                    dev, minSpacing, NOMINAL_CARRIER_BW_HZ + MIN_GUARD_BAND_HZ, NOMINAL_CARRIER_BW_HZ, MIN_GUARD_BAND_HZ);
            }
        }
    }
}

/* Main runtime loop for the RadioManager. */

void RadioManager::runtimeLoop()
{
    while (m_running) {
        ChannelState* channel = nullptr;
        size_t rxDev = 0U;
        size_t txDev = 0U;

        {
            std::lock_guard<std::mutex> lock(m_stateLock);
            if (m_primaryModemValid) {
                auto primaryIt = m_channels.find(m_primaryModemId);
                if (primaryIt != m_channels.end())
                    channel = &primaryIt->second;
            }

            if (channel == nullptr && !m_channels.empty()) {
                auto primaryIt = m_channels.begin();
                m_primaryModemId = primaryIt->first;
                m_primaryModemValid = true;
                channel = &primaryIt->second;
            }

            if (channel != nullptr) {
                rxDev = std::min(channel->rxDevice, m_devices.empty() ? 0U : m_devices.size() - 1U);
                txDev = std::min(channel->txDevice, m_devices.empty() ? 0U : m_devices.size() - 1U);
            }
        }

        if (channel == nullptr) {
            ::usleep(10000U);
            continue;
        }

        const double srRx = (rxDev < m_devices.size()) ? std::max(1.0, m_devices[rxDev].sampleRate) : MODEM_SAMPLE_RATE;
        const size_t hwSamples = std::max<size_t>(PROCESS_BLOCK_SAMPLES,
            static_cast<size_t>(std::llround(srRx * (static_cast<double>(BLOCK_MS) / 1000.0))));

        std::vector<std::complex<float>> channelBuffer(hwSamples, std::complex<float>(0.0f, 0.0f));

#if defined(HAS_SOAPYSDR)
        if (rxDev < m_devices.size()) {
            if (!readSoapyRx(rxDev, channelBuffer))
                std::fill(channelBuffer.begin(), channelBuffer.end(), std::complex<float>(0.0f, 0.0f));
        }
#endif

        float peak = 0.0f;
        uint64_t clips = 0U;

        {
            std::lock_guard<std::mutex> lock(channel->lock);

            const uint32_t srInt = static_cast<uint32_t>(std::max(1LL, std::llround(srRx)));
            const uint32_t modemRate = static_cast<uint32_t>(MODEM_SAMPLE_RATE);
            const uint32_t g = std::gcd(srInt, modemRate);
            const uint32_t num = std::max(1U, modemRate / g);
            const uint32_t den = std::max(1U, srInt / g);
            size_t controlDelay = ((hwSamples * LATENCY_BLOCKS) * static_cast<size_t>(num)) / static_cast<size_t>(std::max(1U, den));
            controlDelay += FDUDC_FILTER_LEN;
            controlDelay = std::max<size_t>(1U, controlDelay);

            if (!channel->fdudc || channel->fdudcNum != num || channel->fdudcDen != den || channel->controlDelaySamples != controlDelay) {
                channel->fdudc = std::make_unique<FDUDC>(num, den, 0, 1, 0, 1, FDUDC_FILTER_LEN, 0.45f);
                channel->fdudcNum = num;
                channel->fdudcDen = den;
                channel->controlDelaySamples = controlDelay;
                channel->delayedControl.assign(channel->controlDelaySamples, 0U);
                channel->prevRx = std::complex<float>(0.0f, 0.0f);
            }

            for (size_t i = 0U; i < channelBuffer.size(); i++) {
                const std::complex<float> lo = std::polar(1.0f, -channel->rxNcoPhase);
                channel->rxNcoPhase = wrapPhase(channel->rxNcoPhase + channel->rxNcoStep);
                channelBuffer[i] *= lo;
            }

            channel->fdudc->process(channelBuffer, [channel](std::complex<float> rxIqSample) {
                float discr = 0.0f;
                if (channel->prevRx.real() != 0.0f || channel->prevRx.imag() != 0.0f)
                    discr = std::arg(rxIqSample * std::conj(channel->prevRx));
                channel->prevRx = rxIqSample;

                discr *= (4096.0f / PI_F);
                if (channel->rxInvert)
                    discr = -discr;

                if (channel->delayedControl.size() < channel->controlDelaySamples)
                    channel->delayedControl.push_back(0U);

                uint8_t control = 0U;
                if (!channel->delayedControl.empty()) {
                    control = channel->delayedControl.front();
                    channel->delayedControl.pop_front();
                }

                if (channel->rxSamples.size() >= MAX_RX_QUEUE_SAMPLES) {
                    channel->rxSamples.pop_front();
                    channel->rxControl.pop_front();
                    channel->rxRssi.pop_front();
                }

                channel->rxSamples.push_back(clampQ15(discr));
                channel->rxControl.push_back(control);
                channel->rxRssi.push_back(clampRssiFromIq(rxIqSample));

                int16_t txSample = 0;
                uint8_t txControl = 0U;
                bool haveTxSample = false;
                if (!channel->txSamples.empty()) {
                    txSample = channel->txSamples.front();
                    channel->txSamples.pop_front();

                    txControl = channel->txControl.front();
                    channel->txControl.pop_front();
                    haveTxSample = true;
                }

                if (channel->txInvert)
                    txSample = static_cast<int16_t>(-txSample);

                channel->txPhaseWord += static_cast<int32_t>(txSample) * FM_DEVIATION;

                channel->delayedControl.push_back(txControl);
                if (!haveTxSample)
                    return std::complex<float>(0.0f, 0.0f);

                const float phase = static_cast<float>(channel->txPhaseWord) * (PI_F / 2147483648.0f);
                const float amp = std::max(0.0f, std::min(1.0f, static_cast<float>(channel->rfPower) / 255.0f));
                return std::polar(amp, phase);
            });

        }

        for (auto& s : channelBuffer) {
            const float mag = std::abs(s);
            peak = std::max(peak, mag);
            if (mag > 1.0f) {
                clips++;
                s /= mag;
            }
        }

        DeviceState* devState = (txDev < m_devices.size()) ? &m_devices[txDev] : nullptr;
        if (devState != nullptr) {
            devState->peakComposite = std::max(devState->peakComposite, peak);
            devState->clipSamples += clips;

            const uint64_t nowMs = monotonicMs();
            if (devState->lastDiagLogMs == 0U || (nowMs - devState->lastDiagLogMs) >= DEVICE_DIAG_INTERVAL_MS) {
                ::LogInfoEx(LOG_SDR, "RF dev%zu STATUS, peak=%.3f, clips=%llu",
                    txDev, devState->peakComposite, static_cast<unsigned long long>(devState->clipSamples));

                devState->peakComposite = 0.0f;
                devState->clipSamples = 0U;
                devState->lastDiagLogMs = nowMs;
            }
        }

#if defined(HAS_SOAPYSDR)
        if (txDev < m_devices.size())
            (void)writeSoapyTx(txDev, channelBuffer);
#endif
    }
}

/* Creates a default device state with standard parameters. */

RadioManager::DeviceState RadioManager::makeDefaultDevice() const
{
    DeviceState dev;
    dev.args = "";
    dev.sampleRate = 960000.0;
    dev.rxGain = 0.0;
    dev.txGain = 0.0;
    dev.rxBandwidth = 0.0;
    dev.txBandwidth = 0.0;
    dev.freqCorrPpm = 0.0;
    dev.rxCenterOffsetHz = 0.0;
    dev.txCenterOffsetHz = 0.0;
    dev.rxCenterHz = 0.0;
    dev.txCenterHz = 0.0;
    dev.rxAntenna = "";
    dev.txAntenna = "";
    dev.clockSource = "";
    dev.timeSource = "";
    dev.rxGainElement = "";
    dev.txGainElement = "";
#if defined(HAS_SOAPYSDR)
    dev.soapyDevice = nullptr;
    dev.rxStream = nullptr;
    dev.txStream = nullptr;
    dev.timestamped = false;
    dev.streamActive = false;
    dev.txTimeNs = 0;
    dev.txLatencyNs = 0;
    dev.lastRxTimeNs = 0;
    dev.lastRxTimeValid = false;
#endif
    dev.hwToModemDecim = 1U;
    dev.modemToHwInterp = 1U;
    dev.lastDiagLogMs = 0U;
    dev.clipSamples = 0U;
    dev.peakComposite = 0.0f;
    dev.minCarrierSpacingHz = 0.0;
    dev.occupiedBandwidthHz = 0.0;
    dev.guardBandViolated = false;
    return dev;
}

/* Retrieves a pointer to the ChannelState for a given modem ID. */

RadioManager::ChannelState* RadioManager::getChannel(uint8_t modemId)
{
    std::lock_guard<std::mutex> lock(m_stateLock);

    if (m_primaryModemValid) {
        auto primaryIt = m_channels.find(m_primaryModemId);
        if (primaryIt != m_channels.end())
            return &primaryIt->second;
    }

    auto it = m_channels.find(modemId);
    if (it != m_channels.end()) {
        m_primaryModemId = modemId;
        m_primaryModemValid = true;
        return &it->second;
    }

    if (m_channels.empty())
        return nullptr;

    m_primaryModemId = m_channels.begin()->first;
    m_primaryModemValid = true;
    return &m_channels.begin()->second;
}

#if defined(HAS_SOAPYSDR)
/* Starts a SoapySDR device for the given device index. */

bool RadioManager::startSoapyDevice(size_t devIdx)
{
    if (devIdx >= m_devices.size())
        return false;

    DeviceState& dev = m_devices[devIdx];
    if (dev.soapyDevice != nullptr)
        return true;

    try {
        SoapySDR::Kwargs kwargs = SoapySDR::KwargsFromString(dev.args);
        dev.soapyDevice = SoapySDR::Device::make(kwargs);
        if (dev.soapyDevice == nullptr)
            throw std::runtime_error("SoapySDR::Device::make returned null");

        dev.soapyDevice->setSampleRate(SOAPY_SDR_RX, 0U, dev.sampleRate);
        dev.soapyDevice->setSampleRate(SOAPY_SDR_TX, 0U, dev.sampleRate);
        dev.soapyDevice->setFrequencyCorrection(SOAPY_SDR_RX, 0U, dev.freqCorrPpm);
        dev.soapyDevice->setFrequencyCorrection(SOAPY_SDR_TX, 0U, dev.freqCorrPpm);

        if (dev.rxBandwidth > 0.0)
            dev.soapyDevice->setBandwidth(SOAPY_SDR_RX, 0U, dev.rxBandwidth);
        if (dev.txBandwidth > 0.0)
            dev.soapyDevice->setBandwidth(SOAPY_SDR_TX, 0U, dev.txBandwidth);

        if (!dev.clockSource.empty())
            dev.soapyDevice->setClockSource(dev.clockSource);
        if (!dev.timeSource.empty())
            dev.soapyDevice->setTimeSource(dev.timeSource);

        if (!dev.rxAntenna.empty())
            dev.soapyDevice->setAntenna(SOAPY_SDR_RX, 0U, dev.rxAntenna);
        if (!dev.txAntenna.empty())
            dev.soapyDevice->setAntenna(SOAPY_SDR_TX, 0U, dev.txAntenna);

        if (dev.rxGain > 0.0) {
            if (!dev.rxGainElement.empty())
                dev.soapyDevice->setGain(SOAPY_SDR_RX, 0U, dev.rxGainElement, dev.rxGain);
            else
                dev.soapyDevice->setGain(SOAPY_SDR_RX, 0U, dev.rxGain);
        }
        if (dev.txGain > 0.0) {
            if (!dev.txGainElement.empty())
                dev.soapyDevice->setGain(SOAPY_SDR_TX, 0U, dev.txGainElement, dev.txGain);
            else
                dev.soapyDevice->setGain(SOAPY_SDR_TX, 0U, dev.txGain);
        }

        dev.rxStream = dev.soapyDevice->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, {0U});
        dev.txStream = dev.soapyDevice->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, {0U});
        if (dev.rxStream == nullptr || dev.txStream == nullptr)
            throw std::runtime_error("setupStream failed");

        dev.soapyDevice->activateStream(dev.rxStream);
        dev.soapyDevice->activateStream(dev.txStream);

        dev.streamActive = true;
        dev.timestamped = dev.soapyDevice->hasHardwareTime();
        const size_t hwSamples = std::max<size_t>(PROCESS_BLOCK_SAMPLES, static_cast<size_t>(std::llround(std::max(1.0, dev.sampleRate) * (static_cast<double>(BLOCK_MS) / 1000.0))));
        dev.txLatencyNs = static_cast<long long>(std::llround((1e9 * static_cast<double>(hwSamples * LATENCY_BLOCKS)) / std::max(1.0, dev.sampleRate)));
        dev.lastRxTimeNs = 0LL;
        dev.lastRxTimeValid = false;
        dev.txTimeNs = dev.timestamped ? (dev.soapyDevice->getHardwareTime() + dev.txLatencyNs) : 0LL;

        ::LogInfoEx(LOG_SDR, "Soapy device %zu started, args='%s', sampleRate=%.0f, rxCenter=%.0f, txCenter=%.0f, timestamped=%s",
            devIdx, dev.args.c_str(), dev.sampleRate, dev.rxCenterHz, dev.txCenterHz, dev.timestamped ? "yes" : "no");

        return true;
    }
    catch (const std::exception& e) {
        ::LogWarning(LOG_SDR, "Failed to start Soapy device %zu (%s), falling back to simulated RF loopback: %s",
            devIdx, dev.args.c_str(), e.what());
        stopSoapyDevice(devIdx);
        return false;
    }
}

/* Stops a SoapySDR device for the given device index. */

void RadioManager::stopSoapyDevice(size_t devIdx)
{
    if (devIdx >= m_devices.size())
        return;

    DeviceState& dev = m_devices[devIdx];
    if (dev.soapyDevice == nullptr)
        return;

    try {
        if (dev.rxStream != nullptr) {
            dev.soapyDevice->deactivateStream(dev.rxStream);
            dev.soapyDevice->closeStream(dev.rxStream);
        }
        if (dev.txStream != nullptr) {
            dev.soapyDevice->deactivateStream(dev.txStream);
            dev.soapyDevice->closeStream(dev.txStream);
        }
    }
    catch (...) {
        // best effort cleanup
    }

    SoapySDR::Device::unmake(dev.soapyDevice);
    dev.soapyDevice = nullptr;
    dev.rxStream = nullptr;
    dev.txStream = nullptr;
    dev.streamActive = false;
    dev.timestamped = false;
    dev.txTimeNs = 0LL;
    dev.txLatencyNs = 0LL;
    dev.lastRxTimeNs = 0LL;
    dev.lastRxTimeValid = false;
}

/* Stops all SoapySDR devices that are currently active. */

void RadioManager::stopAllSoapyDevices()
{
    for (size_t i = 0U; i < m_devices.size(); i++)
        stopSoapyDevice(i);
}

/* Reads samples from a SoapySDR device's RX stream. */

bool RadioManager::readSoapyRx(size_t devIdx, std::vector<std::complex<float>>& iq)
{
    if (devIdx >= m_devices.size())
        return false;

    DeviceState& dev = m_devices[devIdx];
    if (dev.soapyDevice == nullptr || dev.rxStream == nullptr)
        return false;

    void* buffs[1] = {iq.data()};
    int flags = 0;
    long long timeNs = 0LL;
    const int ret = dev.soapyDevice->readStream(dev.rxStream, buffs, static_cast<int>(iq.size()), flags, timeNs, SOAPY_STREAM_TIMEOUT_US);
    if (ret <= 0) {
        ::LogWarning(LOG_SDR, "Soapy RX readStream failed on device %zu: %d (%s)", devIdx, ret, SoapySDR_errToStr(ret));
        return false;
    }

    if (dev.timestamped && (flags & SOAPY_SDR_HAS_TIME) != 0) {
        dev.lastRxTimeNs = timeNs;
        dev.lastRxTimeValid = true;
    }

    if (static_cast<size_t>(ret) < iq.size()) {
        std::fill(iq.begin() + ret, iq.end(), std::complex<float>(0.0f, 0.0f));
    }

    return true;
}

/* Writes samples to a SoapySDR device's TX stream. */

bool RadioManager::writeSoapyTx(size_t devIdx, const std::vector<std::complex<float>>& iq)
{
    if (devIdx >= m_devices.size())
        return false;

    DeviceState& dev = m_devices[devIdx];
    if (dev.soapyDevice == nullptr || dev.txStream == nullptr)
        return false;

    const double sr = std::max(1.0, dev.sampleRate);
    long long txTimeNs = dev.txTimeNs;
    if (dev.timestamped && dev.lastRxTimeValid) {
        const long long rxAnchoredTx = dev.lastRxTimeNs + dev.txLatencyNs;
        if (txTimeNs < rxAnchoredTx)
            txTimeNs = rxAnchoredTx;
    }

    size_t written = 0U;
    uint32_t timeoutRetries = 0U;

    // write the samples to the hardware in a loop until all samples are written, handling timeouts and errors 
    // appropriately, if the device supports timestamping, we include the timestamp for each write to ensure accurate 
    // timing of the transmitted samples, if a timeout occurs, we retry the write operation up to a maximum number of 
    // retries before giving up and dropping the block, which allows us to handle transient issues with the hardware 
    // without causing prolonged disruptions to the transmission
    while (written < iq.size()) {
        const void* buffs[1] = { iq.data() + written };
        int flags = 0;
        long long timeNs = 0LL;

        if (dev.timestamped) {
            flags |= SOAPY_SDR_HAS_TIME;
            timeNs = txTimeNs;
        }

        const size_t remaining = iq.size() - written;
        const int ret = dev.soapyDevice->writeStream(dev.txStream, buffs, static_cast<int>(remaining), flags, timeNs, SOAPY_STREAM_TIMEOUT_US);
        if (ret == SOAPY_SDR_TIMEOUT) {
            timeoutRetries++;
            if (timeoutRetries >= MAX_TX_TIMEOUT_RETRIES) {
                ::LogWarning(LOG_SDR, "Soapy TX timeout persisted on device %zu (%u retries), dropping block", devIdx, timeoutRetries);
                return false;
            }
            ::usleep(1000U);
            continue;
        }

        if (ret <= 0) {
            ::LogWarning(LOG_SDR, "Soapy TX writeStream failed on device %zu: %d (%s)", devIdx, ret, SoapySDR_errToStr(ret));
            return false;
        }
        timeoutRetries = 0U;

        written += static_cast<size_t>(ret);
        if (dev.timestamped) {
            const long long deltaNs = static_cast<long long>((1e9 * static_cast<double>(ret)) / sr);
            txTimeNs += deltaNs;
        }
    }

    if (dev.timestamped)
        dev.txTimeNs = txTimeNs;

    return true;
}
#endif
