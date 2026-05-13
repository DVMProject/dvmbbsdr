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

#include <gnuradio/analog/frequency_modulator_fc.h>
#include <gnuradio/analog/quadrature_demod_cf.h>
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
#endif

#include <osmosdr/sink.h>
#include <osmosdr/source.h>

#include <gnuradio/gr_complex.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#if defined(HAS_GNURADIO_ZEROMQ)
#include <zmq.h>
#endif

using namespace radio;

namespace {
    // Modem-domain sample rate expected by protocol DSP paths.
    constexpr double kModemSampleRate = 24000.0;
    constexpr size_t kRxBurstSamples = 4096U;
    constexpr size_t kQueueCapSamples = 65536U;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kChannelBandwidth = 12500.0;
    constexpr double kTransitionBandwidth = 2500.0;

    /**
     * @brief Utility function to escape a string for JSON encoding.
     * @param in Input string to escape.
     * @returns Escaped string safe for JSON encoding.
     */
    static std::string jsonEscape(const std::string& in)
    {
        std::string out;
        out.reserve(in.size());
        for (const char c : in) {
            switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                out += c;
                break;
            }
        }
        return out;
    }

    /**
     * @brief GNU Radio sync block to consume RX samples from a flowgraph and push to a modem queue.
     */
    struct ChannelQueue {
        std::mutex mtx;

        // FM path: real int16_t samples (12-bit unsigned range, matching IO ADC/DAC convention)
        std::deque<int16_t> rx;
        std::deque<int16_t> tx;

        // IQ path: complex float samples (IQ_CQPSK mode)
        std::deque<gr_complex> rxIQ;
        std::deque<gr_complex> txIQ;

        uint32_t rxFreq = 0U;
        uint32_t txFreq = 0U;
        uint8_t rfPower = 0U;
        uint32_t rxDeviceIndex = 0U;
        uint32_t txDeviceIndex = 0U;

        // FM path activity flag: true while QueueSource consumed real TX samples during
        // the latest scheduler work cycle. Used to gate FM carrier output when idle.
        std::atomic<bool> fmCarrierOn{false};

        radio::ModulationMode mode = radio::ModulationMode::FM_C4FM;
    };

    /**
     * @brief GNU Radio sync block to consume TX samples from a modem queue and push to a flowgraph.
     */
    class QueueSink : public gr::sync_block {
    public:
        typedef std::shared_ptr<QueueSink> sptr;

        /**
         * @brief Factory method to create a new QueueSink instance.
         * @param queue Shared pointer to the channel queue to push RX samples into.
         * @returns Shared pointer to the created QueueSink instance.
         */
        static sptr make(std::shared_ptr<ChannelQueue> queue)
        {
            return gnuradio::get_initial_sptr(new QueueSink(std::move(queue)));
        }

        /**
         * @brief Consumes samples from the flowgraph and pushes to the channel queue.
         * @param noutput_items Number of samples in the input buffer.
         * @param input_items Vector of input buffers.
         * @param output_items Vector of output buffers (not used).
         * @returns Number of input samples consumed.
         */
        int work(int noutput_items, gr_vector_const_void_star& input_items, 
            gr_vector_void_star& output_items) override
        {
            (void)output_items;

            const auto* in = static_cast<const float*>(input_items[0]);
            std::lock_guard<std::mutex> lock(m_queue->mtx);

            // copy samples from input buffer to channel queue, popping old samples if queue exceeds capacity
            for (int i = 0; i < noutput_items; ++i) {
                float v = std::max(-1.0f, std::min(1.0f, in[i] / 2.0f));
                int32_t sample = static_cast<int32_t>(std::lround((v * 2047.0f) + 2048.0f));
                sample = std::max(0, std::min(4095, sample));

                if (m_queue->rx.size() >= kQueueCapSamples)
                    m_queue->rx.pop_front();
                m_queue->rx.push_back(static_cast<int16_t>(sample));
            }

            return noutput_items;
        }

    private:
        /**
         * @brief Initializes a new instance of the QueueSink class.
         */
        explicit QueueSink(std::shared_ptr<ChannelQueue> queue) :
            gr::sync_block("dvmbb_rx_queue_sink",
                           gr::io_signature::make(1, 1, sizeof(float)),
                           gr::io_signature::make(0, 0, 0)),
            m_queue(std::move(queue))
        {
            /* stub */
        }

        std::shared_ptr<ChannelQueue> m_queue;
    };

    /**
     * @brief GNU Radio sync block to consume complex RX I/Q samples from a flowgraph and push to a modem IQ queue.
     * Used in IQ_CQPSK modulation mode; receives gr_complex samples directly from the channelizer.
     */
    class QueueSinkIQ : public gr::sync_block {
    public:
        typedef std::shared_ptr<QueueSinkIQ> sptr;

        /**
         * @brief Factory method to create a new QueueSinkIQ instance.
         * @param queue Shared pointer to the channel queue to push RX IQ samples into.
         * @returns Shared pointer to the created QueueSinkIQ instance.
         */
        static sptr make(std::shared_ptr<ChannelQueue> queue)
        {
            return gnuradio::get_initial_sptr(new QueueSinkIQ(std::move(queue)));
        }

        /**
         * @brief Consumes complex samples from the flowgraph and pushes to the channel IQ queue.
         */
        int work(int noutput_items, gr_vector_const_void_star& input_items,
            gr_vector_void_star& output_items) override
        {
            (void)output_items;

            const auto* in = static_cast<const gr_complex*>(input_items[0]);
            std::lock_guard<std::mutex> lock(m_queue->mtx);

            for (int i = 0; i < noutput_items; ++i) {
                if (m_queue->rxIQ.size() >= kQueueCapSamples)
                    m_queue->rxIQ.pop_front();
                m_queue->rxIQ.push_back(in[i]);
            }

            return noutput_items;
        }

    private:
        /**
         * @brief Initializes a new instance of the QueueSinkIQ class.
         */
        explicit QueueSinkIQ(std::shared_ptr<ChannelQueue> queue) :
            gr::sync_block("dvmbb_rx_iq_queue_sink",
                           gr::io_signature::make(1, 1, sizeof(gr_complex)),
                           gr::io_signature::make(0, 0, 0)),
            m_queue(std::move(queue))
        {
            /* stub */
        }

        std::shared_ptr<ChannelQueue> m_queue;
    };

    /**
     * @brief GNU Radio sync block to produce TX samples from a modem queue and push to a flowgraph.
     */
    class QueueSource : public gr::sync_block {
    public:
        typedef std::shared_ptr<QueueSource> sptr;

        /**
         * @brief Factory method to create a new QueueSource instance.
         * @param queue Shared pointer to the channel queue to pull TX samples from.
         * @returns Shared pointer to the created QueueSource instance.
         */
        static sptr make(std::shared_ptr<ChannelQueue> queue)
        {
            return gnuradio::get_initial_sptr(new QueueSource(std::move(queue)));
        }

        /**
         * @brief Produces samples for the flowgraph by consuming from the channel queue.
         * @param noutput_items Number of samples to produce in the output buffer.
         * @param input_items Vector of input buffers (not used).
         * @param output_items Vector of output buffers.
         * @returns Number of output samples produced.
         */
        int work(int noutput_items, gr_vector_const_void_star& input_items, 
            gr_vector_void_star& output_items) override
        {
            (void)input_items;

            auto* out = static_cast<float*>(output_items[0]);
            std::lock_guard<std::mutex> lock(m_queue->mtx);
            bool hadData = false;

            // copy samples from channel queue to output buffer, pushing zeros if queue is empty
            for (int i = 0; i < noutput_items; ++i) {
                if (!m_queue->tx.empty()) {
                    int16_t s = m_queue->tx.front();
                    m_queue->tx.pop_front();
                    hadData = true;

                    float centered = (static_cast<float>(s) - 2048.0f) / 2048.0f;
                    out[i] = centered;
                } else {
                    out[i] = 0.0f;
                }
            }

            m_queue->fmCarrierOn.store(hadData, std::memory_order_relaxed);

            return noutput_items;
        }

    private:
        /**
         * @brief Initializes a new instance of the QueueSource class.
         */
        explicit QueueSource(std::shared_ptr<ChannelQueue> queue) :
            gr::sync_block("dvmbb_tx_queue_source",
                           gr::io_signature::make(0, 0, 0),
                           gr::io_signature::make(1, 1, sizeof(float))),
            m_queue(std::move(queue))
        {
        }

        std::shared_ptr<ChannelQueue> m_queue;
    };

    /**
     * @brief GNU Radio sync block to produce complex TX I/Q samples from a modem IQ queue and push to a flowgraph.
     * Used in IQ_CQPSK modulation mode; outputs gr_complex samples directly to the rotator and SDR sink,
     * bypassing FM modulation.
     */
    class QueueSourceIQ : public gr::sync_block {
    public:
        typedef std::shared_ptr<QueueSourceIQ> sptr;

        /**
         * @brief Factory method to create a new QueueSourceIQ instance.
         * @param queue Shared pointer to the channel queue to pull TX IQ samples from.
         * @returns Shared pointer to the created QueueSourceIQ instance.
         */
        static sptr make(std::shared_ptr<ChannelQueue> queue)
        {
            return gnuradio::get_initial_sptr(new QueueSourceIQ(std::move(queue)));
        }

        /**
         * @brief Produces complex samples for the flowgraph by consuming from the channel IQ queue.
         */
        int work(int noutput_items, gr_vector_const_void_star& input_items,
            gr_vector_void_star& output_items) override
        {
            (void)input_items;

            auto* out = static_cast<gr_complex*>(output_items[0]);
            std::lock_guard<std::mutex> lock(m_queue->mtx);

            for (int i = 0; i < noutput_items; ++i) {
                if (!m_queue->txIQ.empty()) {
                    out[i] = m_queue->txIQ.front();
                    m_queue->txIQ.pop_front();
                } else {
                    out[i] = gr_complex(0.0f, 0.0f);
                }
            }

            return noutput_items;
        }

    private:
        /**
         * @brief Initializes a new instance of the QueueSourceIQ class.
         */
        explicit QueueSourceIQ(std::shared_ptr<ChannelQueue> queue) :
            gr::sync_block("dvmbb_tx_iq_queue_source",
                           gr::io_signature::make(0, 0, 0),
                           gr::io_signature::make(1, 1, sizeof(gr_complex))),
            m_queue(std::move(queue))
        {
        }

        std::shared_ptr<ChannelQueue> m_queue;
    };

    /**
     * @brief GNU Radio sync block that gates FM-modulated complex output.
     *
     * frequency_modulator_fc emits a constant unit phasor when input is 0.0f,
     * which appears as an unwanted always-on carrier. This block suppresses
     * that by outputting zeros whenever the FM queue source is idle.
     */
    class FMCarrierGate : public gr::sync_block {
    public:
        typedef std::shared_ptr<FMCarrierGate> sptr;

        /**
         * @brief Factory method to create a new FMCarrierGate instance.
         * @param queue Shared pointer to the channel queue to check FM TX activity.
         * @returns Shared pointer to the created FMCarrierGate instance.
         */
        static sptr make(std::shared_ptr<ChannelQueue> queue)
        {
            return gnuradio::get_initial_sptr(new FMCarrierGate(std::move(queue)));
        }

        /**
         * @brief Produces complex samples for the flowgraph by gating the input based on FM TX activity.
         * If the channel queue indicates that FM TX samples were produced during the latest scheduler cycle,
         * this block forwards the input samples (the FM-modulated signal). Otherwise, it outputs zeros to suppress 
         * the carrier.
         * @param noutput_items Number of samples to produce in the output buffer.
         * @param input_items Vector of input buffers (FM-modulated complex samples).
         * @param output_items Vector of output buffers.
         * @returns Number of output samples produced.
         */
        int work(int noutput_items, gr_vector_const_void_star& input_items,
            gr_vector_void_star& output_items) override
        {
            const auto* in = static_cast<const gr_complex*>(input_items[0]);
            auto* out = static_cast<gr_complex*>(output_items[0]);

            // check the channel queue's FM carrier activity flag to determine whether to pass through the 
            // input or output zeros.
            const bool carrierOn = m_queue->fmCarrierOn.load(std::memory_order_relaxed);
            if (!carrierOn) {
                for (int i = 0; i < noutput_items; ++i)
                    out[i] = gr_complex(0.0f, 0.0f);
            } else {
                for (int i = 0; i < noutput_items; ++i)
                    out[i] = in[i];
            }

            return noutput_items;
        }

    private:
        /**
         * @brief Initializes a new instance of the FMCarrierGate class.
         * @param queue Shared pointer to the channel queue to check FM TX activity.
         */
        explicit FMCarrierGate(std::shared_ptr<ChannelQueue> queue) :
            gr::sync_block("dvmbb_tx_fm_carrier_gate",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
            m_queue(std::move(queue))
        {
            /* stub */
        }

        std::shared_ptr<ChannelQueue> m_queue;
    };
}

// ---------------------------------------------------------------------------
//  Structure Definition
// ---------------------------------------------------------------------------

/**
 * @brief Implementation of the radio manager internals.
 * Encapsulates the internal state and logic for managing SDR devices, channels, and flowgraphs. This structure
 * is out of the norm for typical structuring of DVM projects, but we're following a GNUradio esque format here.
 * We're also doing this to keep include complexity to a bare minimum, such taht the GNUradio dependancies only really
 * need to be included in this .cpp file and not in the header, which is included by other parts of the codebase 
 * that we want to keep decoupled from GNUradio.
 */
struct RadioManager::RMInternals {
    /**
     * @brief Internal structure to hold runtime state for each SDR device.
     */
    struct ChannelGraph {
        std::shared_ptr<ChannelQueue> queue;

        // RX channelizer (shared by both FM and IQ paths)
        gr::filter::freq_xlating_fir_filter_ccf::sptr rxXlate;

        // FM RX path: quadrature demod → gain → real queue sink
        gr::analog::quadrature_demod_cf::sptr rxDemod;
        gr::blocks::multiply_const_ff::sptr rxGain;
        QueueSink::sptr rxOut;

        // IQ RX path: complex samples from channelizer directly to IQ queue sink
        QueueSinkIQ::sptr rxOutIQ;

        // FM TX path: real queue source → resampler → FM modulator → rotator
        QueueSource::sptr txIn;
        gr::filter::rational_resampler_fff::sptr txResamp;
        gr::analog::frequency_modulator_fc::sptr txFm;
        FMCarrierGate::sptr txFmGate;

        // TX rotator (shared by both FM and IQ paths)
        gr::blocks::rotator_cc::sptr txShift;

        // IQ TX path: complex queue source → rotator (skips resampler and FM modulator)
        QueueSourceIQ::sptr txInIQ;
    };

    /**
     * @brief Internal structure to hold runtime state for each SDR device.
     */
    struct DeviceRuntime {
        std::string args;
        bool canRx = true;
        bool canTx = true;
        double sampleRate = 960000.0;
        double rxGain = 30.0;
        double txGain = 30.0;
        double freqCorrPpm = 0.0;
        std::string rxAntenna;
        std::string txAntenna;
        double rxBandwidth = 0.0;
        double txBandwidth = 0.0;
    #if defined(HAS_GNURADIO_ZEROMQ)
        // Optional headless debug tap for exporting wideband RX IQ via ZeroMQ PUB.
        std::string rxIqTapAddress;
        std::string rxIqTapTopic;
    #endif
        double rxCenter = 0.0;
        double txCenter = 0.0;
        uint32_t decimInterpRatio = 1U;

        gr::top_block_sptr tb;
        osmosdr::source::sptr src;
        osmosdr::sink::sptr sink;
    #if defined(HAS_GNURADIO_ZEROMQ)
        gr::zeromq::pub_sink::sptr rxIqTap;
    #endif

        gr::blocks::add_cc::sptr txSum;
        std::vector<float> channelTaps;
        std::vector<uint8_t> rxModemIds;
        std::vector<uint8_t> txModemIds;
        std::unordered_map<uint8_t, ChannelGraph> channelGraphs;
    };

    /**
     * @brief Internal structure to hold the modem sample queues for an SDR channel.
     */
    struct DeviceTopology {
        std::set<uint8_t> rxModems;
        std::set<uint8_t> txModems;
    };

    std::mutex lock;
    bool running = false;
    std::string runtimeStatusPubAddress;
    std::string runtimeStatusPubTopic;
    uint64_t runtimeStatusLastPublishMs = 0U;
#if defined(HAS_GNURADIO_ZEROMQ)
    void* runtimeStatusPubCtx = nullptr;
    void* runtimeStatusPubSock = nullptr;
#endif

    std::vector<DeviceRuntime> devices;
    std::unordered_map<uint8_t, std::shared_ptr<ChannelQueue>> channels;

    // Scratch buffer for FM RX dequeue (real int16_t samples)
    std::vector<int16_t> scratch;
    // Scratch buffer for IQ RX dequeue (interleaved int16_t I,Q pairs)
    std::vector<int16_t> scratchIQ;

    bool debug;

    /**
     * @brief Builds a JSON string representing the current runtime status of the RadioManager, including device states 
     *  and channel configurations.
     * @returns JSON string with the current runtime status.
     */
    std::string buildRuntimeStatusJson() const
    {
        const uint64_t updatedMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

        std::string payload;
        payload.reserve(2048U);

        /*
        ** bryanb: we're hand crafting this JSON -- yea yea I should use the json.h library but because this
        **  is basically just for debug and I want to avoid extra dependencies and complexity, I'm just going to 
        **  build the string manually
        */

        payload += "{\n";
        payload += "  \"running\": ";
        payload += running ? "true" : "false";
        payload += ",\n";
        payload += "  \"updatedMs\": ";
        payload += std::to_string(updatedMs);
        payload += ",\n";
        payload += "  \"devices\": [\n";

        // iterate over devices and include their current center/sample-rate and active RX/TX modem counts. this is 
        // used by tools/zmq_fft_view.py to keep the FFT display aligned with dynamic retunes and modem activity
        for (size_t devIdx = 0; devIdx < devices.size(); ++devIdx) {
            const auto& dev = devices[devIdx];
            payload += "    {\n";
            payload += "      \"index\": ";
            payload += std::to_string(devIdx);
            payload += ",\n";
            payload += "      \"sampleRate\": ";
            payload += std::to_string(dev.sampleRate);
            payload += ",\n";
            payload += "      \"rxCenter\": ";
            payload += std::to_string(dev.rxCenter);
            payload += ",\n";
            payload += "      \"txCenter\": ";
            payload += std::to_string(dev.txCenter);
            payload += ",\n";
            payload += "      \"rxActive\": ";
            payload += !dev.rxModemIds.empty() ? "true" : "false";
            payload += ",\n";
            payload += "      \"txActive\": ";
            payload += !dev.txModemIds.empty() ? "true" : "false";
#if defined(HAS_GNURADIO_ZEROMQ)
            payload += ",\n";
            payload += "      \"rxIqTapAddress\": \"";
            payload += jsonEscape(dev.rxIqTapAddress);
            payload += "\",\n";
            payload += "      \"rxIqTapTopic\": \"";
            payload += jsonEscape(dev.rxIqTapTopic);
            payload += "\"\n";
#else
            payload += "\n";
#endif
            payload += "    }";
            if ((devIdx + 1U) < devices.size())
                payload += ",";
            payload += "\n";
        }

        payload += "  ]\n";
        payload += "}\n";

        return payload;
    }

#if defined(HAS_GNURADIO_ZEROMQ)
    /**
     * @brief Closes the ZeroMQ context and socket used for publishing runtime status updates.
     */
    void closeRuntimeStatusPublisher()
    {
        if (runtimeStatusPubSock != nullptr) {
            ::zmq_close(runtimeStatusPubSock);
            runtimeStatusPubSock = nullptr;
        }
        if (runtimeStatusPubCtx != nullptr) {
            ::zmq_ctx_term(runtimeStatusPubCtx);
            runtimeStatusPubCtx = nullptr;
        }
    }

    /**
     * @brief Ensures that the ZeroMQ publisher socket for runtime status updates is initialized and bound to the 
     * configured address. If the socket is already initialized, this function does nothing. If the socket is not 
     * initialized, it attempts to create a new ZeroMQ context and PUB socket, bind it to the configured address, and 
     * logs the result. If any step fails, it cleans up resources and returns false.
     * @returns true if the runtime status publisher is ready to use, false otherwise.
     */
    bool ensureRuntimeStatusPublisher()
    {
        if (runtimeStatusPubAddress.empty())
            return false;

        if (runtimeStatusPubSock != nullptr)
            return true;

        runtimeStatusPubCtx = ::zmq_ctx_new();
        if (runtimeStatusPubCtx == nullptr) {
            ::LogWarning(LOG_SDR, "Unable to create runtime status ZMQ context");
            return false;
        }

        runtimeStatusPubSock = ::zmq_socket(runtimeStatusPubCtx, ZMQ_PUB);
        if (runtimeStatusPubSock == nullptr) {
            ::LogWarning(LOG_SDR, "Unable to create runtime status ZMQ PUB socket");
            closeRuntimeStatusPublisher();
            return false;
        }

        const int rc = ::zmq_bind(runtimeStatusPubSock, runtimeStatusPubAddress.c_str());
        if (rc != 0) {
            ::LogWarning(LOG_SDR, "Unable to bind runtime status ZMQ PUB socket to %s", runtimeStatusPubAddress.c_str());
            closeRuntimeStatusPublisher();
            return false;
        }

        ::LogInfoEx(LOG_SDR, "Radio runtime status ZMQ PUB: %s%s%s",
            runtimeStatusPubAddress.c_str(),
            runtimeStatusPubTopic.empty() ? "" : " topic=",
            runtimeStatusPubTopic.empty() ? "" : runtimeStatusPubTopic.c_str());

        return true;
    }

    /**
     * @brief Publishes a runtime status update message with the given payload string to the configured ZeroMQ PUB socket. 
     * If the publisher is not initialized, this function does nothing.
     * @param payload The string payload to publish as the runtime status update message.
     */
    void publishRuntimeStatus(const std::string& payload)
    {
        if (!ensureRuntimeStatusPublisher())
            return;

        if (!runtimeStatusPubTopic.empty()) {
            ::zmq_send(runtimeStatusPubSock, runtimeStatusPubTopic.data(), runtimeStatusPubTopic.size(), ZMQ_SNDMORE);
        }

        ::zmq_send(runtimeStatusPubSock, payload.data(), payload.size(), 0);
    }
#endif

    /**
     * @brief Builds and publishes a runtime status update message with the current state of the RadioManager. This 
     * function is intended to be called periodically (e.g. every second) to provide up-to-date status information to 
     * external monitoring tools via ZeroMQ. If the publisher is not configured or fails to initialize, this function 
     * will simply return without publishing.
     */
    void publishRuntimeStatus()
    {
        if (!debug)
            return;

#if !defined(HAS_GNURADIO_ZEROMQ)
        return;
#else
        const std::string payload = buildRuntimeStatusJson();
        if (!runtimeStatusPubAddress.empty())
            this->publishRuntimeStatus(payload);
        runtimeStatusLastPublishMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
#endif
    }

    /**
     * @brief Helper function to publish a runtime status update if the configured interval has elapsed since the last 
     * publish. This is intended to be called periodically (e.g. in the main loop) to ensure that status updates are 
     * published at a regular cadence without needing to track timing externally. If the publisher is not configured 
     * or fails to initialize, this function will simply return without publishing.
     * @param intervalMs The minimum interval in milliseconds between status updates. If the last publish was less than 
     * this interval ago, this function will return without publishing. Default is 1000 ms
     */
    void publishRuntimeStatusHeartbeat(uint64_t intervalMs = 1000U)
    {
        if (!debug)
            return;

#if !defined(HAS_GNURADIO_ZEROMQ)
        (void)intervalMs;
        return;
#else
        if (runtimeStatusPubAddress.empty())
            return;

        const uint64_t nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

        if ((nowMs - runtimeStatusLastPublishMs) < intervalMs)
            return;

        publishRuntimeStatus();
#endif
    }

    /**
     * @brief Helper to check if the SDR driver string contains a specific driver name.
     * @param args The SDR driver argument string.
     * @param driver The driver name to check for.
     * @returns true if the driver name is found in the args string, false otherwise.
     */
    static bool hasDriverArg(const std::string& args, const std::string& driver)
    {
        const std::string key = "driver=" + driver;
        return args.find(key) != std::string::npos;
    }

    /**
     * @brief Helper to probe the capabilities of an SDR driver string for RX and TX support.
     * @param args The SDR driver argument string.
     * @param[out] canRx Flag set to true if the driver supports RX, false otherwise.
     * @param[out] canTx Flag set to true if the driver supports TX, false otherwise.
     */
    static void probeDeviceCapabilities(const std::string& args, bool& canRx, bool& canTx)
    {
        canRx = true;
        canTx = true;

        if (args.empty())
            return;

        try {
            auto src = osmosdr::source::make(args);
            canRx = (src != nullptr);
        } catch (...) {
            canRx = false;
        }

        try {
            auto sink = osmosdr::sink::make(args);
            canTx = (sink != nullptr);
        } catch (...) {
            canTx = false;
        }
    }

    /**
     * @brief Helper to parse a frequency from a YAML node, supporting both numeric and string formats.
     * @param node The YAML node containing the frequency value.
     * @return The parsed frequency in Hz, or 0 if parsing fails.
     */
    static uint32_t parseRxDeviceIndex(yaml::Node modemNode)
    {
        yaml::Node rfNode = modemNode["radio"];
        const uint32_t fallback = rfNode["device"].as<uint32_t>(0U);
        return rfNode["rxDevice"].as<uint32_t>(fallback);
    }

    /**
     * @brief Helper to parse a frequency from a YAML node, supporting both numeric and string formats.
     * @param node The YAML node containing the frequency value.
     * @return The parsed frequency in Hz, or 0 if parsing fails.
     */
    static uint32_t parseTxDeviceIndex(yaml::Node modemNode)
    {
        yaml::Node rfNode = modemNode["radio"];
        const uint32_t fallback = rfNode["device"].as<uint32_t>(0U);
        return rfNode["txDevice"].as<uint32_t>(fallback);
    }

    /**
     * @brief Helper to parse a frequency from a YAML node, supporting both numeric and string formats.
     * @param node The YAML node containing the frequency value.
     * @return The parsed frequency in Hz, or 0 if parsing fails.
     */
    bool isRxActive(const std::shared_ptr<ChannelQueue>& ch) const
    {
        return ch->rxFreq > 0U;
    }

    /**
     * @brief Helper to check if a channel has an active TX frequency.
     * @param ch The channel queue to check.
     * @return true if the channel has a valid TX frequency, false otherwise.
     */
    bool isTxActive(const std::shared_ptr<ChannelQueue>& ch) const
    {
        return ch->txFreq > 0U;
    }

    /**
     * @brief Collects the current modem-to-device topology based on the channel configurations.
     * @param[out] outByDevice Vector to populate with the modem sets for each device.
     * @return true if the topology was successfully collected, false if there was an error (e.g. invalid device index).
     */
    bool collectTopology(std::vector<DeviceTopology>& outByDevice) const
    {
        outByDevice.clear();
        outByDevice.resize(devices.size());

        // iterate through channels and assign modem IDs to the appropriate device sets based on their 
        // configured RX/TX device indices
        for (const auto& kv : channels) {
            uint8_t modemId = kv.first;
            const auto& ch = kv.second;

            if (ch->rxDeviceIndex >= devices.size() || ch->txDeviceIndex >= devices.size())
                return false;

            if (isRxActive(ch))
                outByDevice[ch->rxDeviceIndex].rxModems.insert(modemId);

            if (isTxActive(ch))
                outByDevice[ch->txDeviceIndex].txModems.insert(modemId);
        }

        return true;
    }

    /**
     * @brief Computes the center frequency for a set of modems on a given path (RX or TX) based on their configured frequencies.
     * @param modemIds The set of modem IDs to compute the center for.
     * @param rxPath True to compute for RX path, false for TX path.
     * @param[out] center The computed center frequency in Hz.
     */
    void computeCenter(const std::set<uint8_t>& modemIds, bool rxPath, double& center)
    {
        center = 0.0;
        if (modemIds.empty())
            return;

        // compute center as average of modem frequencies on this path
        for (uint8_t modemId : modemIds) {
            const auto& ch = channels.at(modemId);
            center += static_cast<double>(rxPath ? ch->rxFreq : ch->txFreq);
        }

        center /= static_cast<double>(modemIds.size());
    }

    /**
     * @brief Applies hot retune updates to the SDR flowgraphs based on the current channel configurations, without 
     * performing a full graph rebuild. This method is called when channel frequencies are updated but the overall 
     * modem-to-device topology remains unchanged, allowing for in-place frequency adjustments. It iterates through 
     * each device and its associated channels, computes the new center frequencies, and updates the relevant GNU Radio 
     * blocks (e.g. frequency translators, rotators) with the new frequency settings. The method ensures that only 
     * channels with active frequencies are updated, and it uses appropriate locking to synchronize access to the 
     * flowgraph during updates. This allows for seamless frequency changes without interrupting the overall SDR 
     * operation or requiring a full graph teardown and rebuild, minimizing downtime and disruption to the modem operation.
     */
    void applyHotRetune()
    {
        for (size_t devIdx = 0; devIdx < devices.size(); ++devIdx) {
            auto& dev = devices[devIdx];
            if (!dev.tb)
                continue;
            if (dev.rxModemIds.empty() && dev.txModemIds.empty())
                continue;

            std::set<uint8_t> rxSet(dev.rxModemIds.begin(), dev.rxModemIds.end());
            std::set<uint8_t> txSet(dev.txModemIds.begin(), dev.txModemIds.end());

            // compute new center frequencies for RX and TX paths based on assigned channels
            double newRxCenter = dev.rxCenter;
            double newTxCenter = dev.txCenter;
            if (!rxSet.empty())
                computeCenter(rxSet, true, newRxCenter);
            if (!txSet.empty())
                computeCenter(txSet, false, newTxCenter);

            dev.tb->lock();

            if (dev.src && !rxSet.empty() && std::abs(newRxCenter - dev.rxCenter) > 1.0) {
                dev.src->set_center_freq(newRxCenter, 0U);
                dev.rxCenter = newRxCenter;
            }
            if (dev.sink && !txSet.empty() && std::abs(newTxCenter - dev.txCenter) > 1.0) {
                dev.sink->set_center_freq(newTxCenter, 0U);
                dev.txCenter = newTxCenter;
            }

            // update channel graph blocks for RX and TX paths
            for (uint8_t modemId : dev.rxModemIds) {
                auto graphIt = dev.channelGraphs.find(modemId);
                if (graphIt == dev.channelGraphs.end())
                    continue;

                const auto& ch = channels.at(modemId);
                auto& graph = graphIt->second;

                const double rxOffset = static_cast<double>(ch->rxFreq) - dev.rxCenter;
                if (graph.rxXlate)
                    graph.rxXlate->set_center_freq(rxOffset);
            }

            // update TX shift blocks for any channels on this device, since the TX center change affects 
            // the required shift for all channels
            for (uint8_t modemId : dev.txModemIds) {
                auto graphIt = dev.channelGraphs.find(modemId);
                if (graphIt == dev.channelGraphs.end())
                    continue;

                const auto& ch = channels.at(modemId);
                auto& graph = graphIt->second;
                const double txOffset = static_cast<double>(ch->txFreq) - dev.txCenter;
                if (graph.txShift)
                    graph.txShift->set_phase_inc((2.0 * kPi * txOffset) / dev.sampleRate);
            }

            dev.tb->unlock();
        }

        publishRuntimeStatus();
    }

    /**
     * @brief Helper to check if the modem-to-device topology has changed compared to a given topology snapshot.
     * @param topology The topology snapshot to compare against.
     * @return true if the topology has changed, false if it is the same.
     */
    bool topologyChanged(const std::vector<DeviceTopology>& topology) const
    {
        if (topology.size() != devices.size())
            return true;

        // compare modem sets for each device
        for (size_t devIdx = 0; devIdx < devices.size(); ++devIdx) {
            const auto& dev = devices[devIdx];
            std::set<uint8_t> existingRx(dev.rxModemIds.begin(), dev.rxModemIds.end());
            std::set<uint8_t> existingTx(dev.txModemIds.begin(), dev.txModemIds.end());
            if (existingRx != topology[devIdx].rxModems)
                return true;
            if (existingTx != topology[devIdx].txModems)
                return true;
        }

        return false;
    }

    /**
     * @brief Helper to check if a channel has an active RX frequency.
     * @param ch The channel queue to check.
     * @return true if the channel has a valid RX frequency, false otherwise.
     */
    bool configure(yaml::Node& conf)
    {
        yaml::Node sdrNode = conf["sdr"];
        yaml::Node devicesNode = sdrNode["devices"];

        runtimeStatusPubAddress = sdrNode["runtimeStatusPubAddress"].as<std::string>("");
        runtimeStatusPubTopic = sdrNode["runtimeStatusPubTopic"].as<std::string>("");
    #if !defined(HAS_GNURADIO_ZEROMQ)
        if (!runtimeStatusPubAddress.empty()) {
            ::LogWarning(LOG_SDR, "runtimeStatusPubAddress configured, but this build has no gnuradio-zeromq support");
        }
    #endif

        devices.clear();
        channels.clear();

        if (devicesNode.size() == 0U) {
            DeviceRuntime d;
            devices.push_back(d);
        } else {
            yaml::Node defaults = sdrNode["defaults"];
            const double defaultRate = defaults["sampleRate"].as<double>(960000.0);
            const double defaultRxGain = defaults["rxGain"].as<double>(30.0);
            const double defaultTxGain = defaults["txGain"].as<double>(30.0);
            const double defaultPpm = defaults["freqCorrPpm"].as<double>(0.0);
            const std::string defaultRxAntenna = defaults["rxAntenna"].as<std::string>("");
            const std::string defaultTxAntenna = defaults["txAntenna"].as<std::string>("");
#if defined(HAS_GNURADIO_ZEROMQ)
            const std::string defaultRxIqTapAddress = defaults["rxIqTapAddress"].as<std::string>("");
            const std::string defaultRxIqTapTopic = defaults["rxIqTapTopic"].as<std::string>("");
#endif

            // parse device configurations and probe capabilities
            for (size_t i = 0; i < devicesNode.size(); ++i) {
                yaml::Node& dev = devicesNode[i];

                DeviceRuntime d;
                d.args = dev["args"].as<std::string>("");
                probeDeviceCapabilities(d.args, d.canRx, d.canTx);

                const bool canRxProvided = !dev["canRx"].isNone();
                const bool canTxProvided = !dev["canTx"].isNone();

                if (canRxProvided)
                    d.canRx = dev["canRx"].as<bool>(d.canRx);
                if (canTxProvided)
                    d.canTx = dev["canTx"].as<bool>(d.canTx);

                if (hasDriverArg(d.args, "rtl") && !canTxProvided)
                    d.canTx = false;

                d.sampleRate = dev["sampleRate"].as<double>(defaultRate);
                d.rxGain = dev["rxGain"].as<double>(defaultRxGain);
                d.txGain = dev["txGain"].as<double>(defaultTxGain);
                d.freqCorrPpm = dev["freqCorrPpm"].as<double>(defaultPpm);
                d.rxAntenna = dev["rxAntenna"].as<std::string>(defaultRxAntenna);
                d.txAntenna = dev["txAntenna"].as<std::string>(defaultTxAntenna);
                d.rxBandwidth = dev["rxBandwidth"].as<double>(0.0);
                d.txBandwidth = dev["txBandwidth"].as<double>(0.0);
#if defined(HAS_GNURADIO_ZEROMQ)
                d.rxIqTapAddress = dev["rxIqTapAddress"].as<std::string>(defaultRxIqTapAddress);
                d.rxIqTapTopic = dev["rxIqTapTopic"].as<std::string>(defaultRxIqTapTopic);
#else
                if (!dev["rxIqTapAddress"].isNone()) {
                    ::LogWarning(LOG_SDR, "SDR %zu defines rxIqTapAddress, but this build has no gnuradio-zeromq support", i);
                }
#endif
                devices.push_back(d);
            }
        }

        // parse modem configurations and build channel queues
        yaml::Node modemList = conf["modems"];
        for (size_t i = 0; i < modemList.size(); ++i) {
            auto channel = std::make_shared<ChannelQueue>();
            channel->rxDeviceIndex = parseRxDeviceIndex(modemList[i]);
            channel->txDeviceIndex = parseTxDeviceIndex(modemList[i]);

            if (channel->rxDeviceIndex >= devices.size() || channel->txDeviceIndex >= devices.size()) {
                ::LogError(LOG_SDR, "Modem %zu RF mapping references invalid SDR index (rx=%u, tx=%u)",
                    i + 1U, channel->rxDeviceIndex, channel->txDeviceIndex);
                return false;
            }

            if (!devices[channel->rxDeviceIndex].canRx) {
                ::LogError(LOG_SDR, "Modem %zu RF mapping uses RX-disabled SDR index %u", i + 1U, channel->rxDeviceIndex);
                return false;
            }

            if (!devices[channel->txDeviceIndex].canTx) {
                ::LogError(LOG_SDR, "Modem %zu RF mapping uses TX-disabled SDR index %u", i + 1U, channel->txDeviceIndex);
                return false;
            }

            // parse modulation mode for this modem channel
            std::string modeStr = modemList[i]["radio"]["modulationMode"].as<std::string>("FM_C4FM");
            if (modeStr == "IQ_CQPSK")
                channel->mode = radio::ModulationMode::IQ_CQPSK;
            else
                channel->mode = radio::ModulationMode::FM_C4FM;

            channels[static_cast<uint8_t>(i + 1U)] = channel;
        }

        return true;
    }

    /**
     * @brief Helper to stop and clear all existing flowgraphs and channel graphs, preparing for a fresh rebuild.
     */
    void stopGraphs()
    {
        // stop and clear all existing flowgraphs and channel graphs
        for (auto& d : devices) {
            if (d.tb) {
                d.tb->stop();
                d.tb->wait();
            }

            d.tb.reset();
            d.src.reset();
            d.sink.reset();
#if defined(HAS_GNURADIO_ZEROMQ)
            d.rxIqTap.reset();
#endif
            d.txSum.reset();
            d.channelGraphs.clear();
            d.rxModemIds.clear();
            d.txModemIds.clear();
        }

        running = false;

        if (debug) {
            publishRuntimeStatus();
#if defined(HAS_GNURADIO_ZEROMQ)
            closeRuntimeStatusPublisher();
#endif
        }
    }

    /**
     * @brief Builds the GNU Radio flowgraphs for each SDR device based on the current channel configurations, including
     * setting up the necessary blocks for RX and TX paths, frequency translation, modulation/demodulation, and queue 
     * interfaces. This method is called after the channel configurations have been updated and the topology has changed,
     * requiring a full rebuild of the flowgraphs to reflect the new modem-to-device mappings and frequency settings.
     * It iterates through each device, identifies the active RX and TX channels, computes the center frequencies,
     * and constructs the flowgraph by connecting the appropriate GNU Radio blocks for each channel. The method also
     * includes error handling for invalid configurations (e.g. RX channels on a TX-only device) and ensures that the
     * flowgraphs are properly initialized and started after construction.
     */
    void buildGraphs()
    {
        stopGraphs();

        // iterate through devices and build flowgraphs based on active channels
        for (size_t devIdx = 0; devIdx < devices.size(); ++devIdx) {
            auto& dev = devices[devIdx];

            // identify active RX and TX channels for this device
            std::vector<std::pair<uint8_t, std::shared_ptr<ChannelQueue>>> activeRx;
            std::vector<std::pair<uint8_t, std::shared_ptr<ChannelQueue>>> activeTx;
            for (const auto& kv : channels) {
                const auto& ch = kv.second;
                if (ch->rxDeviceIndex == devIdx && ch->rxFreq > 0U)
                    activeRx.push_back(kv);
                if (ch->txDeviceIndex == devIdx && ch->txFreq > 0U)
                    activeTx.push_back(kv);
            }

            if (activeRx.empty() && activeTx.empty())
                continue;

            // compute center frequencies for RX and TX paths based on active channels
            double rxCenter = 0.0;
            if (!activeRx.empty()) {
                for (const auto& item : activeRx)
                    rxCenter += static_cast<double>(item.second->rxFreq);
                rxCenter /= static_cast<double>(activeRx.size());
            }

            double txCenter = 0.0;
            if (!activeTx.empty()) {
                for (const auto& item : activeTx)
                    txCenter += static_cast<double>(item.second->txFreq);
                txCenter /= static_cast<double>(activeTx.size());
            }

            dev.rxCenter = rxCenter;
            dev.txCenter = txCenter;
            dev.rxModemIds.clear();
            dev.txModemIds.clear();
            dev.channelGraphs.clear();

            if (!activeRx.empty() && !dev.canRx) {
                ::LogError(LOG_SDR, "SDR index %zu is configured TX-only but has active RX channels", devIdx);
                continue;
            }
            if (!activeTx.empty() && !dev.canTx) {
                ::LogError(LOG_SDR, "SDR index %zu is configured RX-only but has active TX channels", devIdx);
                continue;
            }

            // build flowgraph for this device
            dev.tb = gr::make_top_block("dvmbbsdr-rf-" + std::to_string(devIdx));
            if (!activeRx.empty()) {
                dev.src = osmosdr::source::make(dev.args);
                dev.src->set_sample_rate(dev.sampleRate);
                dev.src->set_center_freq(rxCenter, 0U);
                dev.src->set_freq_corr(dev.freqCorrPpm, 0U);
                dev.src->set_gain(dev.rxGain, 0U);
                if (!dev.rxAntenna.empty())
                    dev.src->set_antenna(dev.rxAntenna, 0U);
                if (dev.rxBandwidth > 0.0)
                    dev.src->set_bandwidth(dev.rxBandwidth, 0U);

#if defined(HAS_GNURADIO_ZEROMQ)
                if (!dev.rxIqTapAddress.empty() && debug) {
                    dev.rxIqTap = gr::zeromq::pub_sink::make(sizeof(gr_complex), 1U,
                        const_cast<char*>(dev.rxIqTapAddress.c_str()), 100, false, -1, dev.rxIqTapTopic, true);
                    dev.tb->connect(dev.src, 0, dev.rxIqTap, 0);

                    const std::string endpoint = dev.rxIqTap->last_endpoint();
                    ::LogInfoEx(LOG_SDR, "SDR %zu RX IQ tap enabled (%s)%s%s", devIdx, endpoint.empty() ? dev.rxIqTapAddress.c_str() : endpoint.c_str(),
                        dev.rxIqTapTopic.empty() ? "" : " topic=", dev.rxIqTapTopic.empty() ? "" : dev.rxIqTapTopic.c_str());
                }
#endif
            } else {
                dev.src.reset();
            }

            if (!activeTx.empty()) {
                dev.sink = osmosdr::sink::make(dev.args);
                dev.sink->set_sample_rate(dev.sampleRate);
                dev.sink->set_center_freq(txCenter, 0U);
                dev.sink->set_freq_corr(dev.freqCorrPpm, 0U);
                dev.sink->set_gain(dev.txGain, 0U);
                if (!dev.txAntenna.empty())
                    dev.sink->set_antenna(dev.txAntenna, 0U);
                if (dev.txBandwidth > 0.0)
                    dev.sink->set_bandwidth(dev.txBandwidth, 0U);
                dev.txSum = gr::blocks::add_cc::make();
            } else {
                dev.sink.reset();
                dev.txSum.reset();
            }

            uint32_t ratio = static_cast<uint32_t>(std::lround(dev.sampleRate / kModemSampleRate));
            if (ratio == 0U)
                ratio = 1U;
            dev.decimInterpRatio = ratio;

            dev.channelTaps = gr::filter::firdes::low_pass(1.0, dev.sampleRate,
                kChannelBandwidth, kTransitionBandwidth, gr::fft::window::WIN_HAMMING);

            // build channel graphs for RX and TX paths, connecting the appropriate blocks for each active channel
            for (const auto& item : activeRx) {
                uint8_t modemId = item.first;
                const auto& ch = item.second;
                dev.rxModemIds.push_back(modemId);

                auto it = dev.channelGraphs.find(modemId);
                if (it == dev.channelGraphs.end())
                    it = dev.channelGraphs.emplace(modemId, ChannelGraph()).first;
                auto& graph = it->second;
                graph.queue = ch;

                double rxOffset = static_cast<double>(ch->rxFreq) - rxCenter;
                graph.rxXlate = gr::filter::freq_xlating_fir_filter_ccf::make(
                    static_cast<int>(ratio), dev.channelTaps, rxOffset, dev.sampleRate);

                dev.tb->connect(dev.src, 0, graph.rxXlate, 0);

                if (ch->mode == radio::ModulationMode::IQ_CQPSK) {
                    // IQ mode: pass complex samples from channelizer directly to IQ sink.
                    // FM demodulation is skipped; the protocol engine handles all baseband DSP.
                    graph.rxOutIQ = QueueSinkIQ::make(ch);
                    dev.tb->connect(graph.rxXlate, 0, graph.rxOutIQ, 0);
                } else {
                    // FM mode: demodulate quadrature FM and pass real audio samples to sink.
                    graph.rxDemod = gr::analog::quadrature_demod_cf::make(1.0f);
                    graph.rxGain = gr::blocks::multiply_const_ff::make(2.0f);
                    graph.rxOut = QueueSink::make(ch);
                    dev.tb->connect(graph.rxXlate, 0, graph.rxDemod, 0);
                    dev.tb->connect(graph.rxDemod, 0, graph.rxGain, 0);
                    dev.tb->connect(graph.rxGain, 0, graph.rxOut, 0);
                }
            }

            // for TX, we connect all channels to a common sum block that feeds the sink, so we need to build the 
            // entire graph before connecting to the sink
            size_t inputIdx = 0U;
            for (const auto& item : activeTx) {
                uint8_t modemId = item.first;
                const auto& ch = item.second;
                dev.txModemIds.push_back(modemId);

                auto it = dev.channelGraphs.find(modemId);
                if (it == dev.channelGraphs.end())
                    it = dev.channelGraphs.emplace(modemId, ChannelGraph()).first;
                auto& graph = it->second;
                graph.queue = ch;

                double txOffset = static_cast<double>(ch->txFreq) - txCenter;
                graph.txShift = gr::blocks::rotator_cc::make((2.0 * kPi * txOffset) / dev.sampleRate);

                if (ch->mode == radio::ModulationMode::IQ_CQPSK) {
                    // IQ mode: accept complex samples from modem IQ queue, route through rotator to sink.
                    // FM resampling and modulation are skipped.
                    graph.txInIQ = QueueSourceIQ::make(ch);
                    dev.tb->connect(graph.txInIQ, 0, graph.txShift, 0);
                } else {
                    // FM mode: accept real audio from modem, resample, FM modulate, then shift.
                    graph.txIn = QueueSource::make(ch);
                    graph.txResamp = gr::filter::rational_resampler_fff::make(ratio, 1U);
                    graph.txFm = gr::analog::frequency_modulator_fc::make(static_cast<float>((2.0 * kPi * 1500.0) / dev.sampleRate));
                    graph.txFmGate = FMCarrierGate::make(ch);
                    dev.tb->connect(graph.txIn, 0, graph.txResamp, 0);
                    dev.tb->connect(graph.txResamp, 0, graph.txFm, 0);
                    dev.tb->connect(graph.txFm, 0, graph.txFmGate, 0);
                    dev.tb->connect(graph.txFmGate, 0, graph.txShift, 0);
                }

                dev.tb->connect(graph.txShift, 0, dev.txSum, inputIdx++);
            }

            if (dev.txSum && dev.sink)
                dev.tb->connect(dev.txSum, 0, dev.sink, 0);

            dev.tb->start();
        }

        running = true;
        ::LogInfoEx(LOG_SDR, "Radio runtime started (%zu SDR device(s))", devices.size());
        
        publishRuntimeStatus();
    }
};

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Returns the process-global SDR runtime instance. */

RadioManager& RadioManager::instance()
{
    static RadioManager s_instance;
    return s_instance;
}

/* Initializes the SDR runtime and starts SDR flowgraphs. */

bool RadioManager::initialize(yaml::Node& conf)
{
    std::lock_guard<std::mutex> guard(m_internal->lock);

    if (!m_internal->configure(conf)) {
        ::LogError(LOG_SDR, "Radio runtime configuration failed");
        return false;
    }

    m_internal->buildGraphs();
    return true;
}

/* Stops all flowgraphs and releases SDR resources. */

void RadioManager::shutdown()
{
    std::lock_guard<std::mutex> guard(m_internal->lock);
    m_internal->stopGraphs();
}

/* Updates modem RF channel settings. */

void RadioManager::setChannelRF(uint8_t modemId, uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower)
{
    std::lock_guard<std::mutex> guard(m_internal->lock);

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return;

    auto& ch = it->second;
    {
        std::lock_guard<std::mutex> qlock(ch->mtx);
        ch->rxFreq = rxFreq;
        ch->txFreq = txFreq;
        ch->rfPower = rfPower;
    }

    std::vector<RMInternals::DeviceTopology> topology;
    if (!m_internal->collectTopology(topology)) {
        ::LogError(LOG_SDR, "Invalid SDR topology: modem mapped to out-of-range device");
        return;
    }

    if (!m_internal->running || m_internal->topologyChanged(topology)) {
        m_internal->buildGraphs();
        return;
    }

    m_internal->applyHotRetune();
}

/* Queues modem-domain TX samples for SDR transmission. */

void RadioManager::enqueueTx(uint8_t modemId, const uint8_t* samples, size_t length)
{
    if (samples == nullptr || length < sizeof(int16_t))
        return;

    std::lock_guard<std::mutex> guard(m_internal->lock);

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return;

    auto& ch = it->second;
    std::lock_guard<std::mutex> qlock(ch->mtx);

    // copy samples to channel queue, popping old samples if queue exceeds capacity
    const size_t count = length / sizeof(int16_t);
    const auto* in = reinterpret_cast<const int16_t*>(samples);
    for (size_t i = 0; i < count; ++i) {
        if (ch->tx.size() >= kQueueCapSamples)
            ch->tx.pop_front();
        ch->tx.push_back(in[i]);
    }
}

/* Dequeues modem-domain RX samples from SDR path. */

int RadioManager::dequeueRx(uint8_t modemId, uint8_t*& samples)
{
    samples = nullptr;

    std::lock_guard<std::mutex> guard(m_internal->lock);
    m_internal->publishRuntimeStatusHeartbeat();

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return 0;

    auto& ch = it->second;
    std::lock_guard<std::mutex> qlock(ch->mtx);

    if (ch->rx.empty())
        return 0;

    // copy samples to scratch buffer and pop from queue
    const size_t n = std::min(ch->rx.size(), kRxBurstSamples);
    m_internal->scratch.resize(n);
    for (size_t i = 0; i < n; ++i) {
        m_internal->scratch[i] = ch->rx.front();
        ch->rx.pop_front();
    }

    samples = reinterpret_cast<uint8_t*>(m_internal->scratch.data());
    return static_cast<int>(n * sizeof(int16_t));
}

/* Queues modem-domain I/Q TX samples for SDR transmission. */

void RadioManager::enqueueIQTx(uint8_t modemId, const uint8_t* samples, size_t length)
{
    if (samples == nullptr || length < (2 * sizeof(int16_t)))
        return;

    std::lock_guard<std::mutex> guard(m_internal->lock);

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return;

    auto& ch = it->second;
    std::lock_guard<std::mutex> qlock(ch->mtx);

    // input format: interleaved int16_t I,Q pairs; normalize to gr_complex [-1.0, 1.0]
    const size_t count = length / (2 * sizeof(int16_t));
    const auto* in = reinterpret_cast<const int16_t*>(samples);
    for (size_t i = 0; i < count; ++i) {
        if (ch->txIQ.size() >= kQueueCapSamples)
            ch->txIQ.pop_front();
        const float iVal = static_cast<float>(in[2 * i])     / 32767.0f;
        const float qVal = static_cast<float>(in[2 * i + 1]) / 32767.0f;
        ch->txIQ.push_back(gr_complex(iVal, qVal));
    }
}

/* Dequeues modem-domain I/Q RX samples from SDR path. */

int RadioManager::dequeueIQRx(uint8_t modemId, uint8_t*& samples)
{
    samples = nullptr;

    std::lock_guard<std::mutex> guard(m_internal->lock);
    m_internal->publishRuntimeStatusHeartbeat();

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return 0;

    auto& ch = it->second;
    std::lock_guard<std::mutex> qlock(ch->mtx);

    if (ch->rxIQ.empty())
        return 0;

    // convert gr_complex back to interleaved int16_t I,Q pairs
    const size_t n = std::min(ch->rxIQ.size(), kRxBurstSamples);
    m_internal->scratchIQ.resize(n * 2);
    for (size_t i = 0; i < n; ++i) {
        const gr_complex s = ch->rxIQ.front();
        ch->rxIQ.pop_front();
        m_internal->scratchIQ[2 * i]     = static_cast<int16_t>(std::max(-32767.0f, std::min(32767.0f, s.real() * 32767.0f)));
        m_internal->scratchIQ[2 * i + 1] = static_cast<int16_t>(std::max(-32767.0f, std::min(32767.0f, s.imag() * 32767.0f)));
    }

    samples = reinterpret_cast<uint8_t*>(m_internal->scratchIQ.data());
    return static_cast<int>(n * 2 * sizeof(int16_t));
}

/* Returns the configured modulation mode for a modem channel. */

radio::ModulationMode RadioManager::getChannelMode(uint8_t modemId)
{
    std::lock_guard<std::mutex> guard(m_internal->lock);

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return radio::ModulationMode::FM_C4FM;

    return it->second->mode;
}

/* Returns whether modem TX data is pending in SDR runtime queues. */

bool RadioManager::hasPendingTx(uint8_t modemId)
{
    std::lock_guard<std::mutex> guard(m_internal->lock);

    auto it = m_internal->channels.find(modemId);
    if (it == m_internal->channels.end())
        return false;

    auto& ch = it->second;
    std::lock_guard<std::mutex> qlock(ch->mtx);
    return !ch->tx.empty() || !ch->txIQ.empty();
}

/* Enables or disables debug logging for the RadioManager. */

void RadioManager::setDebug(bool enabled)
{
    std::lock_guard<std::mutex> guard(m_internal->lock);
    m_internal->debug = enabled;
    m_debug = enabled;
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the RadioManager class. */

RadioManager::RadioManager() :
    m_internal(new RMInternals()),
    m_debug(false)
{
    /* stub */
}

/* Finalizes a instance of the RadioManager class.  */

RadioManager::~RadioManager()
{
    shutdown();
    delete m_internal;
    m_internal = nullptr;
}
