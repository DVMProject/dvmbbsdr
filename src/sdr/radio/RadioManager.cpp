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

#include <gnuradio/blocks/add_blk.h>
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/blocks/rotator_cc.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/freq_xlating_fir_filter.h>
#include <gnuradio/filter/rational_resampler.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/top_block.h>

#if defined(HAS_GNURADIO_ZEROMQ)
#include <gnuradio/zeromq/pub_sink.h>
#include <zmq.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <sstream>

#include <osmosdr/sink.h>
#include <osmosdr/source.h>

using namespace radio;

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

#define MAX_CHANNEL_RX_QUEUE_BYTES 96000U
#define MAX_CHANNEL_TX_QUEUE_BYTES 96000U
#define RX_POP_BURST_SAMPLES 480U
#define STATUS_PUBLISH_INTERVAL_MS 500U
#define DIAGNOSTICS_LOG_INTERVAL_MS 5000U

#define MODEM_SAMPLE_RATE 24000.0
#define CHANNELIZER_TARGET_RATE 96000.0
#define RX_DISCRIMINATOR_INPUT_RATE MODEM_SAMPLE_RATE
#define RX_DISCRIMINATOR_DECIMATION 1U

#define C4FM_DEVIATION_HZ 2880.0
#define RX_CHANNEL_LPF_CUTOFF_HZ 14400.0
#define RX_CHANNEL_LPF_TRANSITION_HZ 2880.0
#define RX_DISCRIMINATOR_TARGET_PEAK 1152.0f
#define RX_DISCRIMINATOR_DC_ALPHA 0.0005f
#define RX_RSSI_SCALE 100000000.0f
#define RX_DISCRIMINATOR_BASELINE 1.5f  // Bridge-only gain trim for I/Q discriminator drive into modem RX path

#define TX_FM_DEVIATION 550000

#define CONTROL_DELAY_SAMPLES 96U

#define CHANNEL_MARK_NONE 0x00U

// ---------------------------------------------------------------------------
//  Global Functions
// ---------------------------------------------------------------------------

/**
 * @brief Parses a boolean canTx parameter from the device args string, defaulting to true if not specified.
 * @param args Device args string.
 * @return bool Parsed canTx value.
 */
bool parseCanTx(const std::string& args)
{
    if (args.find("canTx=false") != std::string::npos)
        return false;

    if (args.find("canTx=true") != std::string::npos)
        return true;

    if (args.find("driver=rtl") != std::string::npos)
        return false;

    return true;
}

/**
 * @brief Safely computes a phase increment for a given frequency and sample rate, returning 0.0 for non-positive 
 * sample rates.
 * @param frequencyHz Frequency in Hz for which to compute the phase increment.
 * @param sampleRate Sample rate in samples per second.
 * @return double Phase increment in radians per sample, or 0.0 if the sample
 */
double safePhaseInc(double frequencyHz, double sampleRate)
{
    if (sampleRate <= 0.0)
        return 0.0;

    return (2.0 * M_PI * frequencyHz) / sampleRate;
}

/**
 * @brief Safely converts a floating-point sample rate to an unsigned integer, returning a fallback value for 
 * non-positive rates.
 * @param rate Sample rate in samples per second.
 * @param fallback Fallback value to return if the rate is non-positive or rounds to less than 1.0.
 * @return unsigned Sample rate as an unsigned integer, or the fallback
 */
unsigned asUnsignedRate(double rate, unsigned fallback)
{
    if (rate < 1.0)
        return fallback;

    const double rounded = std::round(rate);
    if (rounded < 1.0)
        return fallback;

    return static_cast<unsigned>(rounded);
}

/**
 * @brief Helper to clamp a floating-point RSSI value to the valid range of 0.0 to 65535.0, which corresponds to 
 * the range of a uint16_t.
 * @param v RSSI value to clamp.
 * @return float Clamped RSSI value.
 */
float clampRssi(float v)
{
    if (v < 0.0f)
        return 0.0f;
    if (v > 65535.0f)
        return 65535.0f;
    return v;
}

/**
 * @brief Helper to clamp a long value to the valid range of int16_t, incrementing a counter if clamping occurs.
 * @param v Value to clamp.
 * @param clampCounter Counter to increment if clamping occurs.
 * @return int16_t Clamped value.
 */
int16_t clampToInt16(long v, uint64_t& clampCounter)
{
    if (v > 32767L) {
        clampCounter++;
        return 32767;
    }

    if (v < -32768L) {
        clampCounter++;
        return -32768;
    }

    return static_cast<int16_t>(v);
}

// ---------------------------------------------------------------------------
//  Externs
// ---------------------------------------------------------------------------

extern uint64_t monotonicMs();

// ---------------------------------------------------------------------------
//  Class Definition
// ---------------------------------------------------------------------------

/**
 * @brief GNU Radio source block that provides TX IQ samples to the SDR runtime for a specific modem channel.
 */
class ModemTxIqSource final : public gr::sync_block {
public:
    using sptr = std::shared_ptr<ModemTxIqSource>;

    /**
     * @brief Factory method to create a new instance of the ModemTxIqSource block.
     * @param manager Pointer to the RadioManager instance for sample queue interaction.
     * @param modemId Modem ID associated with this TX source block.
     * @return sptr Shared pointer to the created ModemTxSource instance.
     */
    static sptr make(RadioManager* manager, uint8_t modemId)
    {
        return std::make_shared<ModemTxIqSource>(manager, modemId);
    }

    /**
     * @brief Initializes a new instance of the ModemTxIqSource class.
     * input and output signatures.
     * @param manager Pointer to the RadioManager instance for sample queue interaction.
     * @param modemId Modem ID associated with this TX source block.
     */
    ModemTxIqSource(RadioManager* manager, uint8_t modemId) :
        gr::sync_block("dvmbbsdr_modem_tx_iq_source",
            gr::io_signature::make(0, 0, 0),
            gr::io_signature::make(1, 1, sizeof(gr_complex))),
        m_manager(manager),
        m_modemId(modemId)
    {
        /* stub */
    }

    /**
     * @brief Overrides the work function of the sync_block to provide TX IQ samples.
     * @param noutput_items Number of output items (samples) to produce.
     * @param input_items Vector of input buffers (not used in this block).
     * @param output_items Vector of output buffers where the block should write its output samples.
     * @return int Number of output items produced, which should be equal to noutput_items
     */
    int work(int noutput_items, gr_vector_const_void_star&, gr_vector_void_star& output_items) override
    {
        gr_complex* out = reinterpret_cast<gr_complex*>(output_items[0]);
        if (m_manager == nullptr || out == nullptr || noutput_items <= 0)
            return 0;

        const bool txActive = m_manager->isChannelTxActive(m_modemId);
        if (!txActive) {
            std::fill_n(out, static_cast<size_t>(noutput_items), gr_complex(0.0f, 0.0f));
            return noutput_items;
        }

        const size_t requested = static_cast<size_t>(noutput_items);
        const size_t got = m_manager->dequeueChannelTxIqSamples(m_modemId, out, requested);
        if (got < requested) {
            std::fill(out + got, out + requested, gr_complex(0.0f, 0.0f));
        }

        return noutput_items;
    }

private:
    RadioManager* m_manager;
    uint8_t m_modemId;
};

// ---------------------------------------------------------------------------
//  Class Definition
// ---------------------------------------------------------------------------

/**
 * @brief GNU Radio sink block that receives RX IQ samples from the GNU Radio flowgraph for a specific modem channel and
 * queues them in the RadioManager for modem processing.
 */
class ModemRxIqSink final : public gr::sync_block {
public:
    using sptr = std::shared_ptr<ModemRxIqSink>;

    /**
     * @brief Factory method to create a new instance of the ModemRxIqSink block.
     * @param manager Pointer to the RadioManager instance for sample queue interaction.
     * @param modemId Modem ID associated with this RX sink block.
     * @return sptr Shared pointer to the created ModemRxSink instance.
     */
    static sptr make(RadioManager* manager, uint8_t modemId)
    {
        return std::make_shared<ModemRxIqSink>(manager, modemId);
    }

    /**
     * @brief Initializes a new instance of the ModemRxIqSink class.
     * Initializing the sync_block with appropriate input and output signatures.
     * @param manager Pointer to the RadioManager instance for sample queue interaction.
     * @param modemId Modem ID associated with this RX sink block.
     */
    ModemRxIqSink(RadioManager* manager, uint8_t modemId) :
        gr::sync_block("dvmbbsdr_modem_rx_iq_sink",
            gr::io_signature::make(1, 1, sizeof(gr_complex)),
            gr::io_signature::make(0, 0, 0)),
        m_manager(manager),
        m_modemId(modemId)
    {
        /* stub */
    }

    /**
     * @brief Overrides the work function of the sync_block to receive RX IQ samples from the GNU Radio flowgraph.
     * @param noutput_items Number of input items (samples) received.
     * @param input_items Vector of input buffers containing the received samples.
     * @param output_items Vector of output buffers (not used in this block).
     * @return int Number of input items processed, which should be equal to noutput_items
     */
    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star&) override
    {
        const gr_complex* in = reinterpret_cast<const gr_complex*>(input_items[0]);
        if (m_manager == nullptr || in == nullptr || noutput_items <= 0)
            return 0;

        m_manager->enqueueChannelRxIqSamples(m_modemId, in, static_cast<size_t>(noutput_items));
        return noutput_items;
    }

private:
    RadioManager* m_manager;
    uint8_t m_modemId;
};

// ---------------------------------------------------------------------------
//  Class Definition
// ---------------------------------------------------------------------------

/**
 * @brief GNU Radio gate block that suppresses channel RF output while TX is inactive.
 */
class ModemTxGate final : public gr::sync_block {
public:
    using sptr = std::shared_ptr<ModemTxGate>;

    /**
     * @brief Factory method to create a new instance of the ModemTxGate block.
     * @param manager Pointer to the RadioManager instance for checking channel TX active state.
     * @param modemId Modem ID associated with this TX gate block.
     * @return sptr Shared pointer to the created ModemTxGate instance.
     */
    static sptr make(RadioManager* manager, uint8_t modemId)
    {
        return std::make_shared<ModemTxGate>(manager, modemId);
    }

    /**
     * @brief Initializes a new instance of the ModemTxGate class. Initializing the sync_block with appropriate input and output
     * signatures. The block will output the input samples unchanged when the channel is active for transmission, and will output zeros when the channel is inactive.
     * @param manager Pointer to the RadioManager instance for checking channel TX active state.
     * @param modemId Modem ID associated with this TX gate block.
     */
    ModemTxGate(RadioManager* manager, uint8_t modemId) :
        gr::sync_block("dvmbbsdr_modem_tx_gate",
            gr::io_signature::make(1, 1, sizeof(gr_complex)),
            gr::io_signature::make(1, 1, sizeof(gr_complex))),
        m_manager(manager),
        m_modemId(modemId)
    {
        /* stub */
    }

    /**
     * @brief Overrides the work function of the sync_block to suppress channel RF output when TX is inactive. If the
     * channel is active for transmission, outputs the input samples unchanged. If the channel is inactive, outputs zeros.
     * @param noutput_items Number of input items (samples) received.
     * @param input_items Vector of input buffers containing the samples to gate.
     * @param output_items Vector of output buffers where the gated samples should be written.
     * @return int Number of input items processed, which should be equal to noutput_items
     */
    int work(int noutput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items) override
    {
        const gr_complex* in = reinterpret_cast<const gr_complex*>(input_items[0]);
        gr_complex* out = reinterpret_cast<gr_complex*>(output_items[0]);
        if (in == nullptr || out == nullptr || noutput_items <= 0)
            return 0;

        const bool txActive = (m_manager != nullptr) && m_manager->isChannelTxActive(m_modemId);
        if (!txActive) {
            std::fill_n(out, static_cast<size_t>(noutput_items), gr_complex(0.0f, 0.0f));
            return noutput_items;
        }

        std::copy_n(in, static_cast<size_t>(noutput_items), out);
        return noutput_items;
    }

private:
    RadioManager* m_manager;
    uint8_t m_modemId;
};

// ---------------------------------------------------------------------------
//  Structure Definition
// ---------------------------------------------------------------------------

/**
 * @brief Internal runtime context for the RadioManager, containing GNU Radio flowgraph and block instances for each 
 * device and channel. This structure is used to manage the state of the flowgraphs and blocks associated with each SDR 
 * device and modem channel, allowing for dynamic reconfiguration and status monitoring.
 */
struct RadioManager::RuntimeContext {
    /**
     * @brief Runtime state for a single SDR device, including its GNU Radio flowgraph, source and sink blocks, and channel
     * processing blocks. This structure encapsulates all the GNU Radio components associated with a specific SDR 
     * device, allowing for organized management of the flowgraph and blocks for that device.
     */
    struct ChannelRuntime {
        gr::filter::freq_xlating_fir_filter_ccf::sptr rxXlate;
        gr::blocks::rotator_cc::sptr txRotator;
    };

    /**
     * @brief Runtime state for a single SDR device, including its GNU Radio flowgraph, source and sink blocks, and channel
     * processing blocks. This structure encapsulates all the GNU Radio components associated with a specific SDR 
     * device, allowing for organized management of the flowgraph and blocks for that device.
     */
    struct DeviceRuntime {
        gr::top_block_sptr tb;
        osmosdr::source::sptr source;
        osmosdr::sink::sptr sink;
#if defined(HAS_GNURADIO_ZEROMQ)
        gr::zeromq::pub_sink::sptr iqTap;
#endif
        std::unordered_map<uint8_t, ChannelRuntime> channels;
        std::vector<gr::basic_block_sptr> keepAlive;
        bool started;

        /**
         * @brief Initializes a new instance of the DeviceRuntime structure with default values. The flowgraph and blocks
         * are not created or started in the constructor, as they will be set up later based on the device configuration.
         */
        DeviceRuntime() : tb(), source(), sink(), channels(), keepAlive(), started(false)
        {
            /* stub */
        }
    };

    std::unordered_map<size_t, DeviceRuntime> devices;
};

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Gets the singleton instance of the RadioManager. */

RadioManager& RadioManager::instance()
{
    static RadioManager s_instance;
    return s_instance;
}

/* Initializes the RadioManager with the given configuration.*/

bool RadioManager::initialize(yaml::Node& conf, bool debug)
{
    shutdown();

    std::lock_guard<std::mutex> lock(m_lock);
    m_debug = debug;

    if (!parseConfig(conf))
        return false;

    recomputeDeviceCenters();
    startRadios();
    startRuntimeStatusPublisher();

    m_initialized = true;

    ::LogInfoEx(LOG_SDR, "RadioManager initialized with %zu SDR device(s), %zu modem channel(s)",
        m_devices.size(), m_channels.size());

    return true;
}

/* Shuts down the RadioManager, stopping all GNU Radio flowgraphs, clearing channel queues, and releasing resources. */

void RadioManager::shutdown()
{
    stopRuntimeStatusPublisher();

    std::lock_guard<std::mutex> lock(m_lock);

    stopRadios();

    m_channels.clear();
    m_devices.clear();
    m_runtimeStatusPubAddress.clear();
    m_runtimeStatusPubTopic.clear();
    m_lastDiagnosticsLogMs = 0U;
    m_initialized = false;
}

/* Sets the debug mode for the RadioManager. */

void RadioManager::setDebug(bool debug)
{
    std::lock_guard<std::mutex> lock(m_lock);
    m_debug = debug;
}

/* Helper to set the RF channel polarity for a specific modem channel. */

void RadioManager::setChannelPolarity(uint8_t modemId, bool rxInvert, bool txInvert)
{
    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);
    ch.rxInvert = rxInvert;
    ch.txInvert = txInvert;
}

/* Helper to set the RF channel parameters for a specific modem channel. */

void RadioManager::setChannelParams(uint8_t modemId, uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower)
{
    std::lock_guard<std::mutex> lock(m_lock);

    ChannelState& ch = ensureChannel(modemId);

    // Auto-detect host-applied RX-only fine offset steps (e.g. +6000 Hz test workflow)
    // and compensate them in the channelizer so the I/Q discriminator stays near zero-CFO.
    if (ch.rxFreq != 0U && ch.txFreq == txFreq) {
        const int64_t deltaRx = static_cast<int64_t>(rxFreq) - static_cast<int64_t>(ch.rxFreq);
        if (std::llabs(deltaRx) > 0LL && std::llabs(deltaRx) <= 20000LL) {
            ch.rxHostFreqOffsetCompHz = static_cast<int32_t>(deltaRx);
            if (m_debug) {
                ::LogInfoEx(LOG_SDR, "Modem %u host RX offset compensation update, deltaHz = %d",
                    static_cast<unsigned>(modemId), ch.rxHostFreqOffsetCompHz);
            }
        }
    }

    ch.rxFreq = rxFreq;
    ch.txFreq = txFreq;
    ch.rfPower = rfPower;

    recomputeDeviceCenters();
    applyRetune();
}

/* Helper to set AFC state for a specific modem channel. */

void RadioManager::setChannelAFC(uint8_t modemId, bool enable, uint8_t afcKI, uint8_t afcKP, uint8_t afcRange)
{
    std::lock_guard<std::mutex> lock(m_lock);

    ChannelState& ch = ensureChannel(modemId);
    ch.afcEnable = enable || (afcRange > 0U);
    ch.afcKI = afcKI;
    ch.afcKP = afcKP;
    ch.afcRange = afcRange;
    ch.afcErrorAccum = 0.0;
    ch.afcIntegrator = 0.0;
    ch.afcSampleCount = 0U;
    ch.rxAfcOffsetHz = 0;

    if (m_debug) {
        ::LogInfoEx(LOG_SDR, "Modem %u AFC config, enable = %u, afcKI = %u, afcKP = %u, afcRange = %u",
            static_cast<unsigned>(modemId), ch.afcEnable ? 1U : 0U, ch.afcKI, ch.afcKP, ch.afcRange);
    }
}

/* Helper to set the TX active state for a specific modem channel. */

void RadioManager::setChannelTxActive(uint8_t modemId, bool active)
{
    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);
    ch.txActive = active;
}

/* Helper to push samples to a channel's TX queue. */

void RadioManager::pushChannelTxSamples(uint8_t modemId, const int16_t* samples, const uint8_t* control, size_t sampleCount)
{
    if (samples == nullptr || sampleCount == 0U)
        return;

    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);

    for (size_t i = 0U; i < sampleCount; ++i) {
        ch.txSampleQueue.push_back(samples[i]);
        ch.txControlQueue.push_back(control != nullptr ? control[i] : CHANNEL_MARK_NONE);
    }

    while (ch.txSampleQueue.size() > (MAX_CHANNEL_TX_QUEUE_BYTES / sizeof(int16_t))) {
        ch.txSampleQueue.pop_front();
        ch.txControlQueue.pop_front();
        ch.droppedTxBytes += sizeof(int16_t);
    }
}

/* Helper to pop samples from a channel's RX queue. */

int RadioManager::popChannelRxSamples(uint8_t modemId, int16_t*& samples, uint8_t*& control, uint16_t*& rssi)
{
    samples = nullptr;
    control = nullptr;
    rssi = nullptr;

    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);
    if (ch.rxSampleQueue.empty())
        return 0;

    const size_t count = std::min(ch.rxSampleQueue.size(), static_cast<size_t>(RX_POP_BURST_SAMPLES));
    if (count == 0U)
        return 0;

    ch.rxSampleScratch.resize(count);
    ch.rxControlScratch.resize(count);
    ch.rxRssiScratch.resize(count);

    for (size_t i = 0U; i < count; ++i) {
        ch.rxSampleScratch[i] = ch.rxSampleQueue.front();
        ch.rxSampleQueue.pop_front();

        ch.rxControlScratch[i] = ch.rxControlQueue.front();
        ch.rxControlQueue.pop_front();

        ch.rxRssiScratch[i] = ch.rxRssiQueue.front();
        ch.rxRssiQueue.pop_front();
    }

    samples = ch.rxSampleScratch.data();
    control = ch.rxControlScratch.data();
    rssi = ch.rxRssiScratch.data();
    return static_cast<int>(count);
}

/* Helper to dequeue IQ samples from a channel's TX queue. */

size_t RadioManager::dequeueChannelTxIqSamples(uint8_t modemId, std::complex<float>* dst, size_t sampleCount)
{
    if (dst == nullptr || sampleCount == 0U)
        return 0U;

    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);

    for (size_t i = 0U; i < sampleCount; ++i) {
        int16_t txSample = 0;
        uint8_t txControl = CHANNEL_MARK_NONE;

        if (!ch.txSampleQueue.empty()) {
            txSample = ch.txSampleQueue.front();
            ch.txSampleQueue.pop_front();

            txControl = ch.txControlQueue.front();
            ch.txControlQueue.pop_front();
            ch.txInputSamples++;
        }
        else {
            ch.txZeroFillSamples++;
        }

        if (ch.txInvert)
            txSample = static_cast<int16_t>(-txSample);

        ch.delayedControl.push_back(txControl);
        ch.maxDelayedControlDepth = std::max(ch.maxDelayedControlDepth, ch.delayedControl.size());
        const size_t maxDelayed = CONTROL_DELAY_SAMPLES + (MAX_CHANNEL_TX_QUEUE_BYTES / sizeof(int16_t));
        while (ch.delayedControl.size() > maxDelayed) {
            ch.delayedControl.pop_front();
        }

        ch.txPhase += static_cast<uint32_t>(static_cast<int32_t>(txSample) * TX_FM_DEVIATION);
        const float ph = static_cast<float>(ch.txPhase) * static_cast<float>(M_PI / 0x80000000UL);
        dst[i] = std::polar(1.0f, ph);
    }

    return sampleCount;
}

/* Helper to enqueue IQ samples to a channel's RX queue. */

void RadioManager::enqueueChannelRxIqSamples(uint8_t modemId, const std::complex<float>* src, size_t sampleCount)
{
    if (src == nullptr || sampleCount == 0U)
        return;

    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);

    // Channels without an assigned RX frequency are not active; ignore their
    // incoming stream so inactive modems do not consume RX queue budget.
    if (ch.rxFreq == 0U)
        return;

    const float discrScale = (static_cast<float>(RX_DISCRIMINATOR_INPUT_RATE) /
        (2.0f * static_cast<float>(M_PI) * C4FM_DEVIATION_HZ)) * RX_DISCRIMINATOR_TARGET_PEAK * RX_DISCRIMINATOR_BASELINE;

    for (size_t i = 0U; i < sampleCount; ++i) {
        const std::complex<float> iq = src[i];
        if (!std::isfinite(iq.real()) || !std::isfinite(iq.imag())) {
            ch.rxRssiClampSamples++;
            continue;
        }

        if (!std::isfinite(ch.prevRxIq.real()) || !std::isfinite(ch.prevRxIq.imag()))
            ch.prevRxIq = std::complex<float>(0.0f, 0.0f);

        const std::complex<float> phaseDelta = iq * std::conj(ch.prevRxIq);
        const float d = std::atan2(phaseDelta.imag(), phaseDelta.real());
        ch.prevRxIq = iq;

        const long discr = static_cast<long>(std::lround(d * discrScale));
        ch.rxDiscrAccum += discr;
        ch.rxDiscrDecimCount++;

        if (ch.rxDiscrDecimCount < RX_DISCRIMINATOR_DECIMATION)
            continue;

        const float discrOutF = static_cast<float>(ch.rxDiscrAccum) /
            static_cast<float>(RX_DISCRIMINATOR_DECIMATION);
        ch.rxDiscrAccum = 0L;
        ch.rxDiscrDecimCount = 0U;

        // Remove slow discriminator DC wander from residual carrier error before modem-domain filters/slicing.
        ch.rxDiscrDcEstimate += RX_DISCRIMINATOR_DC_ALPHA * (discrOutF - ch.rxDiscrDcEstimate);
        const long discrOut = static_cast<long>(std::lround(discrOutF - ch.rxDiscrDcEstimate));

        if (ch.afcEnable) {
            ch.afcErrorAccum += static_cast<double>(discrOut);
            ch.afcSampleCount++;

            constexpr uint32_t AFC_UPDATE_SAMPLES = 38400U;
            if (ch.afcSampleCount >= AFC_UPDATE_SAMPLES) {
                const double avgDiscr = ch.afcErrorAccum / static_cast<double>(ch.afcSampleCount);
                const double hzPerDiscr = static_cast<double>(C4FM_DEVIATION_HZ) /
                    (static_cast<double>(RX_DISCRIMINATOR_TARGET_PEAK) * static_cast<double>(RX_DISCRIMINATOR_BASELINE));
                const double freqErrHz = avgDiscr * hzPerDiscr;
                const double afcRangeHz = std::max(100.0, static_cast<double>(ch.afcRange) * 1000.0);
                const double pGain = static_cast<double>(std::max<uint8_t>(1U, ch.afcKP)) / 32.0;
                const double iGain = static_cast<double>(ch.afcKI) / 2048.0;

                if (std::abs(freqErrHz) < 50.0) {
                    ch.afcErrorAccum = 0.0;
                    ch.afcSampleCount = 0U;
                    return;
                }

                ch.afcIntegrator += freqErrHz;
                if (ch.afcIntegrator > afcRangeHz)
                    ch.afcIntegrator = afcRangeHz;
                else if (ch.afcIntegrator < -afcRangeHz)
                    ch.afcIntegrator = -afcRangeHz;

                double correctionHz = (freqErrHz * pGain) + (ch.afcIntegrator * iGain);
                if (correctionHz > afcRangeHz)
                    correctionHz = afcRangeHz;
                else if (correctionHz < -afcRangeHz)
                    correctionHz = -afcRangeHz;

                double steppedCorrectionHz = correctionHz * 0.0625;
                if (steppedCorrectionHz > 50.0)
                    steppedCorrectionHz = 50.0;
                else if (steppedCorrectionHz < -50.0)
                    steppedCorrectionHz = -50.0;

                const int32_t afcStep = static_cast<int32_t>(std::lround(steppedCorrectionHz));
                const int32_t nextOffset = ch.rxAfcOffsetHz + afcStep;
                const int32_t rangeLimit = static_cast<int32_t>(std::lround(afcRangeHz));
                ch.rxAfcOffsetHz = std::max(-rangeLimit, std::min(rangeLimit, nextOffset));

                if (m_debug) {
                    ::LogInfoEx(LOG_SDR, "Modem %u AFC update, avgDiscr = %0.3f, freqErrHz = %0.3f, corrHz = %0.3f, newOffsetHz = %d",
                        static_cast<unsigned>(modemId), avgDiscr, freqErrHz, correctionHz, ch.rxAfcOffsetHz);
                }

                ch.afcErrorAccum = 0.0;
                ch.afcSampleCount = 0U;
            }
        }

        ch.rxSamples++;

        int16_t sample = clampToInt16(discrOut, ch.rxRssiClampSamples);
        if (ch.rxInvert)
            sample = static_cast<int16_t>(-sample);

        const uint16_t absSample = static_cast<uint16_t>(std::abs(static_cast<int>(sample)));
        if (absSample > ch.rxSampleAbsPeak)
            ch.rxSampleAbsPeak = absSample;

        uint8_t alignedControl = CHANNEL_MARK_NONE;
        if (ch.delayedControl.size() > CONTROL_DELAY_SAMPLES) {
            alignedControl = ch.delayedControl.front();
            ch.delayedControl.pop_front();
            ch.rxControlAlignedSamples++;
        }
        else {
            ch.rxControlDeferredSamples++;
        }

        const float rawRssiFloat = RX_RSSI_SCALE * std::norm(iq);
        if (rawRssiFloat > 65535.0f || rawRssiFloat < 0.0f)
            ch.rxRssiClampSamples++;
        const float rawRssiNonNegative = std::max(rawRssiFloat, 0.0f);
        const float rawRssiBounded = std::min(rawRssiNonNegative, static_cast<float>(std::numeric_limits<uint32_t>::max()));
        const uint32_t rawRssi = static_cast<uint32_t>(std::lround(rawRssiBounded));
        if (ch.rxSamples == 1U) {
            ch.rxRawRssiMin = rawRssi;
            ch.rxRawRssiMax = rawRssi;
        }
        else {
            ch.rxRawRssiMin = std::min(ch.rxRawRssiMin, rawRssi);
            ch.rxRawRssiMax = std::max(ch.rxRawRssiMax, rawRssi);
        }
        const float rssiFloat = clampRssi(rawRssiFloat);
        const uint16_t rxRssi = static_cast<uint16_t>(rssiFloat);

        ch.rxSampleQueue.push_back(sample);
        ch.rxControlQueue.push_back(alignedControl);
        ch.rxRssiQueue.push_back(rxRssi);
    }

    while (ch.rxSampleQueue.size() > (MAX_CHANNEL_RX_QUEUE_BYTES / sizeof(int16_t))) {
        ch.rxSampleQueue.pop_front();
        ch.rxControlQueue.pop_front();
        ch.rxRssiQueue.pop_front();
        ch.droppedRxBytes += sizeof(int16_t);
    }
}

/* Helper to check if a channel is active for transmission. */

bool RadioManager::isChannelTxActive(uint8_t modemId)
{
    std::lock_guard<std::mutex> lock(m_lock);
    ChannelState& ch = ensureChannel(modemId);
    return ch.txActive;
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the RadioManager class.*/

RadioManager::RadioManager() :
    m_lock(),
    m_initialized(false),
    m_debug(false),
    m_channels(),
    m_devices(),
    m_runtimeStatusPubAddress(),
    m_runtimeStatusPubTopic(),
    m_statusThreadStop(true),
    m_statusThread(),
    m_runtime(nullptr),
    m_lastDiagnosticsLogMs(0U)
#if defined(HAS_GNURADIO_ZEROMQ)
    , m_zmqContext(nullptr)
    , m_zmqPubSocket(nullptr)
#endif
{
    /* stub */
}

/* Finalizes an instance of the RadioManager class. */

RadioManager::~RadioManager()
{
    shutdown();
}

/* Helper to ensure a channel state exists for a given modem ID. */

RadioManager::ChannelState& RadioManager::ensureChannel(uint8_t modemId)
{
    auto it = m_channels.find(modemId);
    if (it != m_channels.end())
        return it->second;

    ChannelState ch;
    ch.modemId = modemId;
    ch.rxDevice = 0;
    ch.txDevice = 0;
    ch.rxFreq = 0U;
    ch.txFreq = 0U;
    ch.rfPower = 0U;
    ch.rxInvert = false;
    ch.txInvert = false;
    ch.txActive = false;
    ch.txSampleQueue.clear();
    ch.txControlQueue.clear();
    ch.rxSampleQueue.clear();
    ch.rxControlQueue.clear();
    ch.rxRssiQueue.clear();
    ch.rxSampleScratch.clear();
    ch.rxControlScratch.clear();
    ch.rxRssiScratch.clear();
    ch.txPhase = 0U;
    ch.prevRxIq = std::complex<float>(0.0f, 0.0f);
    ch.delayedControl.assign(CONTROL_DELAY_SAMPLES, CHANNEL_MARK_NONE);
    ch.rxDiscrDecimCount = 0U;
    ch.rxDiscrAccum = 0L;
    ch.rxDiscrDcEstimate = 0.0f;
    ch.rxHostFreqOffsetCompHz = 0;
    ch.afcEnable = false;
    ch.afcKI = 0U;
    ch.afcKP = 0U;
    ch.afcRange = 0U;
    ch.afcErrorAccum = 0.0;
    ch.afcIntegrator = 0.0;
    ch.afcSampleCount = 0U;
    ch.rxAfcOffsetHz = 0;
    ch.txInputSamples = 0U;
    ch.txZeroFillSamples = 0U;
    ch.rxSamples = 0U;
    ch.rxRssiClampSamples = 0U;
    ch.rxRawRssiMin = 0U;
    ch.rxRawRssiMax = 0U;
    ch.rxSampleAbsPeak = 0U;
    ch.rxControlAlignedSamples = 0U;
    ch.rxControlDeferredSamples = 0U;
    ch.maxDelayedControlDepth = ch.delayedControl.size();
    ch.droppedRxBytes = 0U;
    ch.droppedTxBytes = 0U;

    auto result = m_channels.emplace(modemId, std::move(ch));
    return result.first->second;
}

/* Helper to parse the SDR configuration from a YAML node. */

bool RadioManager::parseConfig(yaml::Node& conf)
{
    m_channels.clear();
    m_devices.clear();

    yaml::Node sdrConf = conf["sdr"];
    yaml::Node defaults = sdrConf["defaults"];

    const double defaultSampleRate = defaults["sampleRate"].as<double>(960000.0);
    const double defaultRxGain = defaults["rxGain"].as<double>(0.0);
    const double defaultTxGain = defaults["txGain"].as<double>(0.0);
    const double defaultFreqCorrPpm = defaults["freqCorrPpm"].as<double>(0.0);
    const std::string defaultRxAntenna = defaults["rxAntenna"].as<std::string>("");
    const std::string defaultTxAntenna = defaults["txAntenna"].as<std::string>("");
    const std::string defaultRxIqTapAddress = defaults["rxIqTapAddress"].as<std::string>("");
    const std::string defaultRxIqTapTopic = defaults["rxIqTapTopic"].as<std::string>("");

    m_runtimeStatusPubAddress = sdrConf["runtimeStatusPubAddress"].as<std::string>("");
    m_runtimeStatusPubTopic = sdrConf["runtimeStatusPubTopic"].as<std::string>("");

    yaml::Node devicesNode = sdrConf["devices"];
    if (devicesNode.size() == 0U) {
        DeviceState def;
        def.index = 0U;
        def.args = "";
        def.sampleRate = defaultSampleRate;
        def.rxGain = defaultRxGain;
        def.txGain = defaultTxGain;
        def.freqCorrPpm = defaultFreqCorrPpm;
        def.rxAntenna = defaultRxAntenna;
        def.txAntenna = defaultTxAntenna;
        def.canTx = true;
        def.rxIqTapAddress = defaultRxIqTapAddress;
        def.rxIqTapTopic = defaultRxIqTapTopic;
        def.rxCenter = 0U;
        def.txCenter = 0U;
        def.assignedRxChannels = 0U;
        def.assignedTxChannels = 0U;
        m_devices.push_back(def);
    }
    else {
        for (size_t i = 0U; i < devicesNode.size(); ++i) {
            yaml::Node dev = devicesNode[i];

            DeviceState state;
            state.index = i;
            state.args = dev["args"].as<std::string>("");
            state.sampleRate = dev["sampleRate"].as<double>(defaultSampleRate);
            state.rxGain = dev["rxGain"].as<double>(defaultRxGain);
            state.txGain = dev["txGain"].as<double>(defaultTxGain);
            state.freqCorrPpm = dev["freqCorrPpm"].as<double>(defaultFreqCorrPpm);
            state.rxAntenna = dev["rxAntenna"].as<std::string>(defaultRxAntenna);
            state.txAntenna = dev["txAntenna"].as<std::string>(defaultTxAntenna);
            state.canTx = parseCanTx(state.args);
            state.rxIqTapAddress = dev["rxIqTapAddress"].as<std::string>(defaultRxIqTapAddress);
            state.rxIqTapTopic = dev["rxIqTapTopic"].as<std::string>(defaultRxIqTapTopic);
            state.rxCenter = 0U;
            state.txCenter = 0U;
            state.assignedRxChannels = 0U;
            state.assignedTxChannels = 0U;
            m_devices.push_back(state);
        }
    }

    // parse modem channel configurations and build channel states
    yaml::Node modemList = conf["modems"];
    for (size_t i = 0U; i < modemList.size(); ++i) {
        const uint8_t modemId = static_cast<uint8_t>(i + 1U);
        yaml::Node modemConf = modemList[i];
        yaml::Node radioConf = modemConf["radio"];

        const int sharedDevice = radioConf["device"].as<int>(0);
        const int rxDevice = radioConf["rxDevice"].as<int>(sharedDevice);
        const int txDevice = radioConf["txDevice"].as<int>(sharedDevice);

        ChannelState& ch = ensureChannel(modemId);
        ch.rxDevice = rxDevice;
        ch.txDevice = txDevice;

        if (ch.rxDevice < 0 || static_cast<size_t>(ch.rxDevice) >= m_devices.size()) {
            ::LogWarning(LOG_SDR, "Modem %u references invalid Rx device, remapping to 0, rxDevice = %d", modemId, ch.rxDevice);
            ch.rxDevice = 0;
        }

        if (ch.txDevice < 0 || static_cast<size_t>(ch.txDevice) >= m_devices.size()) {
            ::LogWarning(LOG_SDR, "Modem %u references invalid Tx device, remapping to 0, txDevice = %d", modemId, ch.txDevice);
            ch.txDevice = 0;
        }

        const DeviceState& txDev = m_devices[static_cast<size_t>(ch.txDevice)];
        if (!txDev.canTx && m_debug) {
            ::LogWarning(LOG_SDR, "Modem %u TX assigned to RX-only SDR %d (%s)",
                modemId,
                ch.txDevice,
                txDev.args.c_str());
        }

        ::LogInfoEx(LOG_SDR, "Modem %u BINDING, rxDev = %d, txDev=%d", modemId, ch.rxDevice, ch.txDevice);
    }

    return true;
}

/* Helper to recompute the center frequencies for each SDR device based on the assigned channels. */

void RadioManager::recomputeDeviceCenters()
{
    // for each device, find the min and max assigned RX and TX frequencies to compute a center frequency for tuning
    for (size_t i = 0U; i < m_devices.size(); ++i) {
        DeviceState& dev = m_devices[i];
        dev.assignedRxChannels = 0U;
        dev.assignedTxChannels = 0U;

        uint32_t rxMin = std::numeric_limits<uint32_t>::max();
        uint32_t rxMax = 0U;
        uint32_t txMin = std::numeric_limits<uint32_t>::max();
        uint32_t txMax = 0U;

        // for each channel, check if it's assigned to this device for RX or TX and update the min/max 
        // frequencies accordingly
        for (const auto& entry : m_channels) {
            const ChannelState& ch = entry.second;

            if (ch.rxDevice == static_cast<int>(i) && ch.rxFreq > 0U) {
                dev.assignedRxChannels++;
                rxMin = std::min(rxMin, ch.rxFreq);
                rxMax = std::max(rxMax, ch.rxFreq);
            }

            if (ch.txDevice == static_cast<int>(i) && ch.txFreq > 0U) {
                dev.assignedTxChannels++;
                txMin = std::min(txMin, ch.txFreq);
                txMax = std::max(txMax, ch.txFreq);
            }
        }

        dev.rxCenter = 0U;
        dev.txCenter = 0U;

        if (dev.assignedRxChannels > 0U) {
            const uint64_t avg = (static_cast<uint64_t>(rxMin) + static_cast<uint64_t>(rxMax)) / 2ULL;
            dev.rxCenter = static_cast<uint32_t>(avg);
        }

        if (dev.assignedTxChannels > 0U) {
            const uint64_t avg = (static_cast<uint64_t>(txMin) + static_cast<uint64_t>(txMax)) / 2ULL;
            dev.txCenter = static_cast<uint32_t>(avg);
        }
    }
}

/* Helper to start the GNU Radio flowgraphs for all configured SDR devices. */

void RadioManager::startRadios()
{
    stopRadios();

    m_runtime = std::make_unique<RuntimeContext>();

    std::unordered_map<size_t, std::vector<uint8_t>> rxAssignments;
    std::unordered_map<size_t, std::vector<uint8_t>> txAssignments;

    // build reverse lookup of which channels are assigned to each device for RX and TX
    for (const auto& it : m_channels) {
        const uint8_t modemId = it.first;
        const ChannelState& ch = it.second;

        rxAssignments[static_cast<size_t>(ch.rxDevice)].push_back(modemId);
        txAssignments[static_cast<size_t>(ch.txDevice)].push_back(modemId);
    }

    // for each device, if it has any assigned channels for RX or TX, set up a GNU Radio flowgraph with the appropriate blocks
    for (size_t i = 0U; i < m_devices.size(); ++i) {
        const DeviceState& dev = m_devices[i];
        const auto rxIt = rxAssignments.find(i);
        const auto txIt = txAssignments.find(i);

        const bool hasRx = (rxIt != rxAssignments.end()) && !rxIt->second.empty();
        const bool hasTx = (txIt != txAssignments.end()) && !txIt->second.empty() && dev.canTx;

        if (!hasRx && !hasTx)
            continue;

        RuntimeContext::DeviceRuntime runtime;
        runtime.tb = gr::make_top_block("dvmbbsdr-radio-device-" + std::to_string(i));

        // do we have any RX channels? if so, create the source block and processing chain for each assigned channel
        if (hasRx) {
            runtime.source = osmosdr::source::make(dev.args);
            runtime.source->set_sample_rate(dev.sampleRate);
            const double rxActualRate = runtime.source->get_sample_rate();
            if (std::fabs(rxActualRate - dev.sampleRate) > 1.0) {
                ::LogWarning(LOG_SDR, "SDR %zu RX sample rate coerced, requested = %0.3f, actual = %0.3f", i, dev.sampleRate, rxActualRate);
            }
            else if (m_debug) {
                ::LogInfoEx(LOG_SDR, "SDR %zu RX sample rate, requested = %0.3f, actual = %0.3f", i, dev.sampleRate, rxActualRate);
            }
            runtime.source->set_gain(dev.rxGain);
            runtime.source->set_freq_corr(dev.freqCorrPpm);
            if (!dev.rxAntenna.empty())
                runtime.source->set_antenna(dev.rxAntenna);
            if (dev.rxCenter > 0U)
                runtime.source->set_center_freq(static_cast<double>(dev.rxCenter));

#if defined(HAS_GNURADIO_ZEROMQ)
            if (!dev.rxIqTapAddress.empty()) {
                runtime.iqTap = gr::zeromq::pub_sink::make(sizeof(gr_complex), 1, const_cast<char*>(dev.rxIqTapAddress.c_str()), 
                    100, false, 4, dev.rxIqTapTopic, true);
                runtime.tb->connect(runtime.source, 0, runtime.iqTap, 0);
                runtime.keepAlive.push_back(runtime.iqTap);
                ::LogInfoEx(LOG_SDR, "SDR %zu RX IQ tap enabled (%s)", i, dev.rxIqTapAddress.c_str());
            }
#endif

            const unsigned decim = std::max(1U, asUnsignedRate(dev.sampleRate / CHANNELIZER_TARGET_RATE, 1U));
            const double chanRate = dev.sampleRate / static_cast<double>(decim);

            const std::vector<float> xlateTaps = gr::filter::firdes::low_pass(1.0, dev.sampleRate, RX_CHANNEL_LPF_CUTOFF_HZ,
                RX_CHANNEL_LPF_TRANSITION_HZ, gr::fft::window::WIN_HAMMING);

            if (m_debug) {
                ::LogDebugEx(LOG_SDR, "RadioManager::startRadios()", "SDR %zu RX channelizer, devRate = %0.3f, decim = %u, chanRate = %0.3f, lpfCutoff = %0.3f, lpfTransition = %0.3f, discrRate = %0.3f",
                    i, dev.sampleRate, decim, chanRate, RX_CHANNEL_LPF_CUTOFF_HZ, RX_CHANNEL_LPF_TRANSITION_HZ, RX_DISCRIMINATOR_INPUT_RATE);
            }

            // for each assigned RX channel, create a chain of blocks to translate, resample, and sink IQ to the shim.
            // samples to the modem processing queue
            for (uint8_t modemId : rxIt->second) {
                auto chIt = m_channels.find(modemId);
                if (chIt == m_channels.end())
                    continue;

                const ChannelState& ch = chIt->second;
                const double offsetHz = static_cast<double>(ch.rxFreq) - static_cast<double>(dev.rxCenter) - static_cast<double>(ch.rxHostFreqOffsetCompHz) - static_cast<double>(ch.rxAfcOffsetHz);

                auto xlate = gr::filter::freq_xlating_fir_filter_ccf::make(static_cast<int>(decim), xlateTaps,
                    offsetHz, dev.sampleRate);

                const unsigned chanRateInt = asUnsignedRate(chanRate, 96000U);
                const unsigned discrRateInt = asUnsignedRate(RX_DISCRIMINATOR_INPUT_RATE, static_cast<unsigned>(MODEM_SAMPLE_RATE));
                const unsigned g = std::gcd(chanRateInt, discrRateInt);
                const unsigned interp = discrRateInt / g;
                const unsigned dec = chanRateInt / g;
                auto resamp = gr::filter::rational_resampler_ccf::make(interp, dec);
                auto rxSink = ModemRxIqSink::make(this, modemId);

                runtime.tb->connect(runtime.source, 0, xlate, 0);
                runtime.tb->connect(xlate, 0, resamp, 0);
                runtime.tb->connect(resamp, 0, rxSink, 0);

                RuntimeContext::ChannelRuntime chRuntime;
                chRuntime.rxXlate = xlate;
                runtime.channels[modemId] = chRuntime;

                runtime.keepAlive.push_back(xlate);
                runtime.keepAlive.push_back(resamp);
                runtime.keepAlive.push_back(rxSink);
            }
        }

        // do we have any TX channels? if so, create the sink block and processing chain for each assigned channel, 
        // then combine the outputs to the SDR sink
        if (hasTx) {
            runtime.sink = osmosdr::sink::make(dev.args);
            runtime.sink->set_sample_rate(dev.sampleRate);
            const double txActualRate = runtime.sink->get_sample_rate();
            if (std::fabs(txActualRate - dev.sampleRate) > 1.0) {
                ::LogWarning(LOG_SDR, "SDR %zu TX sample rate coerced, requested = %0.3f, actual = %0.3f", i, dev.sampleRate, txActualRate);
            }
            else if (m_debug) {
                ::LogInfoEx(LOG_SDR, "SDR %zu TX sample rate, requested = %0.3f, actual = %0.3f", i, dev.sampleRate, txActualRate);
            }
            runtime.sink->set_gain(dev.txGain);
            runtime.sink->set_freq_corr(dev.freqCorrPpm);
            if (!dev.txAntenna.empty())
                runtime.sink->set_antenna(dev.txAntenna);
            if (dev.txCenter > 0U)
                runtime.sink->set_center_freq(static_cast<double>(dev.txCenter));

            std::vector<gr::basic_block_sptr> carrierOutputs;

            // for each assigned TX channel, create a chain of blocks to resample, rotate, and sum IQ.
            // from the modem processing queue to produce the final output to the SDR sink
            for (uint8_t modemId : txIt->second) {
                auto chIt = m_channels.find(modemId);
                if (chIt == m_channels.end())
                    continue;

                const ChannelState& ch = chIt->second;
                const double offsetHz = static_cast<double>(ch.txFreq) - static_cast<double>(dev.txCenter);

                auto txSrc = ModemTxIqSource::make(this, modemId);

                const unsigned deviceRateInt = asUnsignedRate(dev.sampleRate, 960000U);
                const unsigned modemRateInt = static_cast<unsigned>(MODEM_SAMPLE_RATE);
                const unsigned g = std::gcd(deviceRateInt, modemRateInt);
                const unsigned interp = deviceRateInt / g;
                const unsigned dec = modemRateInt / g;

                auto up = gr::filter::rational_resampler_ccf::make(interp, dec);
                auto rot = gr::blocks::rotator_cc::make(safePhaseInc(offsetHz, dev.sampleRate));
                auto txGate = ModemTxGate::make(this, modemId);

                runtime.tb->connect(txSrc, 0, up, 0);
                runtime.tb->connect(up, 0, rot, 0);
                runtime.tb->connect(rot, 0, txGate, 0);

                RuntimeContext::ChannelRuntime& chRuntime = runtime.channels[modemId];
                chRuntime.txRotator = rot;

                runtime.keepAlive.push_back(txSrc);
                runtime.keepAlive.push_back(up);
                runtime.keepAlive.push_back(rot);
                runtime.keepAlive.push_back(txGate);

                carrierOutputs.push_back(txGate);
            }

            if (!carrierOutputs.empty()) {
                std::vector<gr::basic_block_sptr> mixed = carrierOutputs;

                while (mixed.size() > 1U) {
                    std::vector<gr::basic_block_sptr> next;
                    next.reserve((mixed.size() + 1U) / 2U);

                    // iteratively add pairs of carrier outputs together until we have a single combined output for the SDR sink
                    for (size_t idx = 0U; idx < mixed.size(); idx += 2U) {
                        if (idx + 1U >= mixed.size()) {
                            next.push_back(mixed[idx]);
                            continue;
                        }

                        auto add = gr::blocks::add_cc::make(1U);
                        runtime.tb->connect(mixed[idx], 0, add, 0);
                        runtime.tb->connect(mixed[idx + 1U], 0, add, 1);
                        runtime.keepAlive.push_back(add);
                        next.push_back(add);
                    }

                    mixed.swap(next);
                }

                gr::basic_block_sptr finalOut = mixed.front();
                if (carrierOutputs.size() > 1U) {
                    const float scale = 1.0f / static_cast<float>(carrierOutputs.size());
                    auto scaler = gr::blocks::multiply_const_cc::make(gr_complex(scale, 0.0f));
                    runtime.tb->connect(finalOut, 0, scaler, 0);
                    runtime.tb->connect(scaler, 0, runtime.sink, 0);
                    runtime.keepAlive.push_back(scaler);
                }
                else {
                    runtime.tb->connect(finalOut, 0, runtime.sink, 0);
                }
            }
        }

        try {
            runtime.tb->start();
            runtime.started = true;
            ::LogInfoEx(LOG_SDR, "SDR %zu flowgraph started (RX:%u TX:%u)",
                i,
                hasRx ? 1U : 0U,
                hasTx ? 1U : 0U);
        }
        catch (...) {
            ::LogError(LOG_SDR, "Failed to start GNU Radio flowgraph for SDR %zu", i);
        }

        m_runtime->devices[i] = std::move(runtime);
    }

    applyRetune();
}

/* Helper to stop the GNU Radio flowgraphs for all SDR devices. */

void RadioManager::stopRadios()
{
    if (!m_runtime)
        return;

    // for each device, if its flowgraph is running, stop it and clear all associated blocks and state to release resources
    for (auto& it : m_runtime->devices) {
        RuntimeContext::DeviceRuntime& dev = it.second;
        if (dev.started && dev.tb) {
            try {
                dev.tb->stop();
                dev.tb->wait();
            }
            catch (...) {
                ::LogWarning(LOG_SDR, "Exception while stopping flowgraph for SDR %zu", it.first);
            }
        }

        dev.started = false;
        dev.keepAlive.clear();
        dev.channels.clear();
        dev.source.reset();
        dev.sink.reset();
#if defined(HAS_GNURADIO_ZEROMQ)
        dev.iqTap.reset();
#endif
        dev.tb.reset();
    }

    m_runtime->devices.clear();
    m_runtime.reset();
}

/* Helper to apply retuning of the SDR devices based on any changes to the channel frequencies. */

void RadioManager::applyRetune()
{
    if (!m_runtime)
        return;

    // for each device, if it has an active flowgraph, update the center frequencies of the source and sink blocks as well as the
    // frequency translation and rotation blocks for each assigned channel to reflect any changes to the channel frequencies
    for (size_t i = 0U; i < m_devices.size(); ++i) {
        auto rtIt = m_runtime->devices.find(i);
        if (rtIt == m_runtime->devices.end())
            continue;

        RuntimeContext::DeviceRuntime& rt = rtIt->second;
        const DeviceState& dev = m_devices[i];

        if (rt.source && dev.rxCenter > 0U)
            rt.source->set_center_freq(static_cast<double>(dev.rxCenter));

        if (rt.sink && dev.txCenter > 0U)
            rt.sink->set_center_freq(static_cast<double>(dev.txCenter));

        // for each assigned channel, compute the new frequency offset from the device center and update the frequency 
        // translation and rotation blocks accordingly
        for (auto& chIt : rt.channels) {
            const uint8_t modemId = chIt.first;
            RuntimeContext::ChannelRuntime& chRt = chIt.second;

            auto stateIt = m_channels.find(modemId);
            if (stateIt == m_channels.end())
                continue;

            const ChannelState& ch = stateIt->second;

            if (chRt.rxXlate) {
                const double offsetHz = static_cast<double>(ch.rxFreq) - static_cast<double>(dev.rxCenter) - static_cast<double>(ch.rxHostFreqOffsetCompHz) - static_cast<double>(ch.rxAfcOffsetHz);
                chRt.rxXlate->set_center_freq(offsetHz);
            }

            if (chRt.txRotator) {
                const double offsetHz = static_cast<double>(ch.txFreq) - static_cast<double>(dev.txCenter);
                chRt.txRotator->set_phase_inc(safePhaseInc(offsetHz, dev.sampleRate));
            }
        }
    }
}

/* Helper to start the runtime status publisher thread.*/

void RadioManager::startRuntimeStatusPublisher()
{
    m_statusThreadStop = false;

#if defined(HAS_GNURADIO_ZEROMQ)
    if (!m_runtimeStatusPubAddress.empty()) {
        m_zmqContext = ::zmq_ctx_new();
        if (m_zmqContext != nullptr) {
            m_zmqPubSocket = ::zmq_socket(m_zmqContext, ZMQ_PUB);
            if (m_zmqPubSocket != nullptr) {
                const int sndHwm = 2;
                const int linger = 0;
                (void)::zmq_setsockopt(m_zmqPubSocket, ZMQ_SNDHWM, &sndHwm, sizeof(sndHwm));
                (void)::zmq_setsockopt(m_zmqPubSocket, ZMQ_LINGER, &linger, sizeof(linger));

                if (::zmq_bind(m_zmqPubSocket, m_runtimeStatusPubAddress.c_str()) != 0) {
                    ::LogWarning(LOG_SDR, "RadioManager runtime status PUB bind failed for %s", m_runtimeStatusPubAddress.c_str());
                    ::zmq_close(m_zmqPubSocket);
                    m_zmqPubSocket = nullptr;
                }
                else {
                    ::LogInfoEx(LOG_SDR, "RadioManager runtime status PUB enabled (%s)", m_runtimeStatusPubAddress.c_str());
                }
            }
        }

        if (m_zmqPubSocket == nullptr && m_zmqContext != nullptr) {
            ::zmq_ctx_term(m_zmqContext);
            m_zmqContext = nullptr;
        }
    }
#else
    if (!m_runtimeStatusPubAddress.empty()) {
        ::LogWarning(LOG_SDR, "runtimeStatusPubAddress is configured, but this build has no gnuradio-zeromq support");
    }
#endif

    m_statusThread = std::thread([this]() {
        while (!m_statusThreadStop.load()) {
            publishRuntimeStatus();
            std::this_thread::sleep_for(std::chrono::milliseconds(STATUS_PUBLISH_INTERVAL_MS));
        }
    });
}

/* Helper to stop the runtime status publisher thread. */

void RadioManager::stopRuntimeStatusPublisher()
{
    m_statusThreadStop = true;

    if (m_statusThread.joinable())
        m_statusThread.join();

#if defined(HAS_GNURADIO_ZEROMQ)
    if (m_zmqPubSocket != nullptr) {
        ::zmq_close(m_zmqPubSocket);
        m_zmqPubSocket = nullptr;
    }

    if (m_zmqContext != nullptr) {
        ::zmq_ctx_term(m_zmqContext);
        m_zmqContext = nullptr;
    }
#endif
}

/* Helper to publish the runtime status of the RadioManager. */

void RadioManager::publishRuntimeStatus()
{
    std::string payload;

    {
        std::lock_guard<std::mutex> lock(m_lock);
        logRuntimeDiagnostics(monotonicMs());
        payload = buildRuntimeStatusJson();
    }

#if defined(HAS_GNURADIO_ZEROMQ)
    if (m_zmqPubSocket != nullptr) {
        if (!m_runtimeStatusPubTopic.empty()) {
            (void)::zmq_send(m_zmqPubSocket, m_runtimeStatusPubTopic.data(), m_runtimeStatusPubTopic.size(), ZMQ_SNDMORE);
        }

        (void)::zmq_send(m_zmqPubSocket, payload.data(), payload.size(), 0);
    }
#else
    (void)payload;
#endif
}

/* Helper to build a JSON string representing the current runtime status of the RadioManager. */

std::string RadioManager::buildRuntimeStatusJson() const
{
    /*
    ** bryanb: we're doing this by hand instead of via json.h because its faster -- if not more dirty
    ** to hand craft the JSON
    */

    std::ostringstream ss;
    ss << "{";
        ss << "\"devices\":[";

    // for each device, include its index, sample rate, current center frequencies, and count of assigned RX and TX channels
    for (size_t i = 0U; i < m_devices.size(); ++i) {
        const DeviceState& dev = m_devices[i];
        if (i > 0U)
            ss << ",";

        ss << "{";
            ss << "\"index\":" << dev.index << ",";
            ss << "\"sampleRate\":" << dev.sampleRate << ",";
            ss << "\"rxCenter\":" << dev.rxCenter << ",";
            ss << "\"txCenter\":" << dev.txCenter << ",";
            ss << "\"assignedRxChannels\":" << dev.assignedRxChannels << ",";
            ss << "\"assignedTxChannels\":" << dev.assignedTxChannels;
        ss << "}";
    }

        ss << "],";
        ss << "\"channels\":[";

    // for each channel, include its modem ID, current RX and TX queue sizes in samples, the depth of the delayed 
    // control queue, the computed control lag in samples, the current TX phase accumulator value, counts of shim input 
    // and zero fill samples for TX and shim samples and statistics for RX
    bool firstChannel = true;
    for (const auto& it : m_channels) {
        const ChannelState& ch = it.second;
        if (!firstChannel)
            ss << ",";
        firstChannel = false;

        const size_t delayedDepth = ch.delayedControl.size();
        const long long controlLag = (delayedDepth > CONTROL_DELAY_SAMPLES) ? static_cast<long long>(delayedDepth - CONTROL_DELAY_SAMPLES) : 0LL;

        ss << "{";
            ss << "\"modemId\":" << static_cast<unsigned>(ch.modemId) << ",";
            ss << "\"txQueueSamples\":" << ch.txSampleQueue.size() << ",";
            ss << "\"rxQueueSamples\":" << ch.rxSampleQueue.size() << ",";
            ss << "\"delayedControlDepth\":" << delayedDepth << ",";
            ss << "\"maxDelayedControlDepth\":" << ch.maxDelayedControlDepth << ",";
            ss << "\"controlLagSamples\":" << controlLag << ",";
            ss << "\"txPhase\":" << ch.txPhase << ",";
            ss << "\"txInputSamples\":" << ch.txInputSamples << ",";
            ss << "\"txZeroFillSamples\":" << ch.txZeroFillSamples << ",";
            ss << "\"rxSamples\":" << ch.rxSamples << ",";
            ss << "\"rxRssiClampSamples\":" << ch.rxRssiClampSamples << ",";
            ss << "\"rxRawRssiMin\":" << ch.rxRawRssiMin << ",";
            ss << "\"rxRawRssiMax\":" << ch.rxRawRssiMax << ",";
            ss << "\"rxSampleAbsPeak\":" << ch.rxSampleAbsPeak << ",";
            ss << "\"rxControlAlignedSamples\":" << ch.rxControlAlignedSamples << ",";
            ss << "\"rxControlDeferredSamples\":" << ch.rxControlDeferredSamples;
        ss << "}";
    }

        ss << "]";
    ss << "}";
    return ss.str();
}

/* Helper to log runtime diagnostics at regular intervals. */

void RadioManager::logRuntimeDiagnostics(uint64_t nowMs)
{
    if (!m_debug)
        return;

    if (m_lastDiagnosticsLogMs != 0U && (nowMs - m_lastDiagnosticsLogMs) < DIAGNOSTICS_LOG_INTERVAL_MS)
        return;

    m_lastDiagnosticsLogMs = nowMs;

    ::LogInfoEx(LOG_SDR, "RadioManager DIAGNOSTICS, devices = %zu, channels = %zu", m_devices.size(), m_channels.size());
    // for each channel, log the current RX and TX queue sizes in bytes and samples, the total dropped bytes for RX and TX, 
    // the assigned device indices and frequencies, the computed center frequencies and offsets from center, and whether 
    // the channel is currently active for transmission
    for (const auto& it : m_channels) {
        const ChannelState& ch = it.second;

        uint32_t rxCenter = 0U;
        uint32_t txCenter = 0U;
        if (ch.rxDevice >= 0 && static_cast<size_t>(ch.rxDevice) < m_devices.size())
            rxCenter = m_devices[static_cast<size_t>(ch.rxDevice)].rxCenter;
        if (ch.txDevice >= 0 && static_cast<size_t>(ch.txDevice) < m_devices.size())
            txCenter = m_devices[static_cast<size_t>(ch.txDevice)].txCenter;

        const int64_t rxOffset = (rxCenter > 0U && ch.rxFreq > 0U) ? static_cast<int64_t>(ch.rxFreq) - static_cast<int64_t>(rxCenter) : 0LL;
        const int64_t txOffset = (txCenter > 0U && ch.txFreq > 0U) ? static_cast<int64_t>(ch.txFreq) - static_cast<int64_t>(txCenter) : 0LL;

        const size_t rxQueueSamples = ch.rxSampleQueue.size();
        const size_t txQueueSamples = ch.txSampleQueue.size();
        const size_t rxQueueBytes = rxQueueSamples * sizeof(int16_t);
        const size_t txQueueBytes = txQueueSamples * sizeof(int16_t);

        const size_t delayedDepth = ch.delayedControl.size();
        const long long controlLag = (delayedDepth > CONTROL_DELAY_SAMPLES) ? static_cast<long long>(delayedDepth - CONTROL_DELAY_SAMPLES) : 0LL;

        ::LogInfoEx(LOG_SDR, "Modem %u DIAGNOSTICS, rxQ = %zuB/%zuS, txQ = %zuB/%zuS, dropRx = %lluB, dropTx = %lluB, rxDev = %d, rxFreq = %u, rxCenter = %u, rxOff = %lld, rxComp = %d, rxAFC = %d, afcOn=%u, txDev = %d, txFreq = %u, txCenter = %u, txOff = %lld, txActive = %u, dlyCtl = %zu, dlyCtlMax = %zu, ctlLag = %lld, txPhase = %u, txIn = %llu, txZero = %llu, rxShim = %llu, rxClamp = %llu, rxRawMin = %u, rxRawMax = %u, rxPeak = %u, rxCtlAlign = %llu, rxCtlDef = %llu",
            ch.modemId, rxQueueBytes, rxQueueSamples, txQueueBytes, txQueueSamples,static_cast<unsigned long long>(ch.droppedRxBytes), static_cast<unsigned long long>(ch.droppedTxBytes), ch.rxDevice,
            ch.rxFreq, rxCenter, static_cast<long long>(rxOffset), ch.rxHostFreqOffsetCompHz, ch.rxAfcOffsetHz, ch.afcEnable ? 1U : 0U, ch.txDevice, ch.txFreq, txCenter, static_cast<long long>(txOffset), ch.txActive ? 1U : 0U,
            delayedDepth, ch.maxDelayedControlDepth, controlLag, ch.txPhase, static_cast<unsigned long long>(ch.txInputSamples),
            static_cast<unsigned long long>(ch.txZeroFillSamples), static_cast<unsigned long long>(ch.rxSamples), static_cast<unsigned long long>(ch.rxRssiClampSamples),
            ch.rxRawRssiMin, ch.rxRawRssiMax, static_cast<unsigned>(ch.rxSampleAbsPeak),
            static_cast<unsigned long long>(ch.rxControlAlignedSamples), static_cast<unsigned long long>(ch.rxControlDeferredSamples));
    }
}
