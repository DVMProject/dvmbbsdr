// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 */
/**
 * @file RadioManager.h
 * @ingroup modem_fw
 * @file RadioManager.cpp
 * @ingroup modem_fw
 *
 * @brief Runtime SDR device and channel manager.
 *
 * This manager owns GNU Radio flowgraphs and provides a queue bridge between
 * virtual modem instances and SDR hardware channels.
 */
#if !defined(__RADIO_MANAGER_H__)
#define __RADIO_MANAGER_H__

#include "Defines.h"
#include "common/yaml/Yaml.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <complex>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace radio
{
    /**
     * @brief Implements the RadioManager class, which manages SDR devices and channels, including their configuration,
     * runtime state, and interaction with GNU Radio flowgraphs. The RadioManager provides methods for initializing and
     * shutting down the SDR runtime environment, configuring channels, and bridging sample queues between modem 
     * instances and SDR hardware. It also handles runtime status publishing and diagnostics logging to facilitate 
     * monitoring and debugging of the SDR operation.
     */
    class DSP_FW_API RadioManager {
    public:
        /**
         * @brief Gets the singleton instance of the RadioManager. This method ensures that only one instance of the
         * RadioManager exists throughout the application, providing a centralized point of access for managing SDR devices
         * and channels. The instance is lazily initialized on the first call to this method, and subsequent calls will return the same instance.
         * @returns RadioManager& Reference to the singleton instance of the RadioManager.
         */
        static RadioManager& instance();

        /**
         * @brief Initializes the RadioManager with the given configuration. This method sets up the SDR devices and channels
         * based on the provided YAML configuration, starts the GNU Radio flowgraphs for each device, and begins publishing 
         * runtime status. It must be called before any other methods to ensure the RadioManager is properly set up.
         * @param conf YAML node containing the configuration for the SDR devices and channels.
         * @returns bool True if initialization was successful, otherwise false.
         */
        bool initialize(yaml::Node& conf);
        /**
         * @brief Shuts down the RadioManager, stopping all GNU Radio flowgraphs, clearing channel queues, and releasing 
         * resources. This method should be called when the SDR runtime is no longer needed to ensure a clean shutdown 
         * and resource release.
         */
        void shutdown();

        /**
         * @brief Sets the debug mode for the RadioManager. When debug mode is enabled, the RadioManager may log additional
         * information about its operation, which can be useful for troubleshooting and development. This method can be called at any time to enable or disable debug logging.
         * @param debug True to enable debug mode, false to disable it.
         */
        void setDebug(bool debug);

        /**
         * @brief Helper to set the RF channel polarity for a specific modem channel. This method allows for dynamic reconfiguration
         * of the channel's RX and TX polarity for RF path integration with different hardware chains.
         * The changes will take effect immediately in the GNU Radio flowgraph.
         * @param modemId Modem ID of the channel to configure.
         * @param rxInvert True to invert the RX polarity, false for normal polarity.
         * @param txInvert True to invert the TX polarity, false for normal polarity.
         */
        void setChannelPolarity(uint8_t modemId, bool rxInvert, bool txInvert);
        /**
         * @brief Helper to set the RF channel parameters for a specific modem channel. This method allows for dynamic reconfiguration
         * of the channel's frequencies, RF power level, and polarity. The changes will take effect immediately in the GNU Radio 
         * flowgraph, allowing for flexible operation of the SDR channels.
         * @param modemId Modem ID of the channel to configure.
         * @param rxFreq Receive frequency in Hz.
         * @param txFreq Transmit frequency in Hz.
         * @param rfPower RF power level hint.
         */
        void setChannelParams(uint8_t modemId, uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower);
        /**
         * @brief Helper to set the TX active state for a specific modem channel. This method is used to indicate whether 
         * the channel is currently active for transmission, which affects whether samples from the channel's TX queue 
         * are sent to the SDR hardware. When a channel is set to active, the GNU Radio flowgraph will output samples 
         * from the channel's TX queue.
         * @param modemId Modem ID of the channel to configure.
         * @param active True to activate TX, false to deactivate.
         */
        void setChannelTxActive(uint8_t modemId, bool active);

        /**
         * @brief Helper to push TX samples and their control flags to a channel queue.
         * @param modemId Modem ID of the channel to which the samples should be pushed.
         * @param samples Buffer containing signed 16-bit discriminator/baseband samples.
         * @param control Optional per-sample control marks. If null, MARK_NONE is assumed.
         * @param sampleCount Number of samples in the input buffers.
         */
        void pushChannelTxSamples(uint8_t modemId, const int16_t* samples, const uint8_t* control, size_t sampleCount);
        /**
         * @brief Helper to pop RX samples from a channel queue.
         * @param modemId Modem ID of the channel from which to pop samples.
         * @param samples Pointer to popped sample vector scratch memory.
         * @param control Pointer to popped control vector scratch memory.
         * @param rssi Pointer to popped RSSI vector scratch memory.
         * @returns int Number of samples popped.
         */
        int popChannelRxSamples(uint8_t modemId, int16_t*& samples, uint8_t*& control, uint16_t*& rssi);

        /**
         * @brief Helper to dequeue IQ samples from a channel's TX queue for GNU Radio.
         * @param modemId Modem ID of the channel from which to dequeue samples.
         * @param dst Buffer to store dequeued complex IQ samples.
         * @param sampleCount Maximum number of samples to dequeue.
         * @returns size_t Number of IQ samples dequeued.
         */
        size_t dequeueChannelTxIqSamples(uint8_t modemId, std::complex<float>* dst, size_t sampleCount);
        /**
         * @brief Helper to enqueue IQ samples from GNU Radio RX into a channel's shim.
         * @param modemId Modem ID of the channel to which the samples should be enqueued.
         * @param src Buffer containing complex IQ samples to enqueue.
         * @param sampleCount Number of samples in the buffer to enqueue.
         */
        void enqueueChannelRxIqSamples(uint8_t modemId, const std::complex<float>* src, size_t sampleCount);

        /**
         * @brief Helper to check if a channel is active for transmission. This method allows modem instances to query the 
         * current TX active state of a specific channel, which can be used to determine whether samples pushed to the 
         * channel's TX queue will be transmitted to the SDR hardware.
         * @param modemId Modem ID of the channel to check.
         * @returns bool True if the channel is active for transmission, false otherwise.
         */
        bool isChannelTxActive(uint8_t modemId);

    private:
        /**
         * @brief Structure to hold the state of a single SDR channel, including its configuration parameters, 
         * sample queues, and runtime statistics. This structure is used internally by the RadioManager to manage the 
         * state of each channel and facilitate the queuing of samples for transmission and reception.
         */
        struct DeviceState {
            size_t index;
            std::string args;
            double sampleRate;
            double rxGain;
            double txGain;
            double freqCorrPpm;
            std::string rxAntenna;
            std::string txAntenna;
            bool canTx;

            std::string rxIqTapAddress;
            std::string rxIqTapTopic;

            uint32_t rxCenter;
            uint32_t txCenter;
            uint32_t assignedRxChannels;
            uint32_t assignedTxChannels;
        };

        /**
         * @brief Structure to hold the state of a single modem channel, including its assigned RX and TX devices,
         * sample queues, and runtime statistics. This structure is used internally by the RadioManager to manage the 
         * state of each modem channel and facilitate the queuing of samples for transmission and reception.
         */
        struct ChannelState {
            uint8_t modemId;
            int rxDevice;
            int txDevice;

            uint32_t rxFreq;
            uint32_t txFreq;
            uint8_t rfPower;
            bool rxInvert;
            bool txInvert;
            bool txActive;

            std::deque<int16_t> txSampleQueue;
            std::deque<uint8_t> txControlQueue;

            std::deque<int16_t> rxSampleQueue;
            std::deque<uint8_t> rxControlQueue;
            std::deque<uint16_t> rxRssiQueue;

            std::vector<int16_t> rxSampleScratch;
            std::vector<uint8_t> rxControlScratch;
            std::vector<uint16_t> rxRssiScratch;

            uint32_t txPhase;
            std::complex<float> prevRxIq;
            std::deque<uint8_t> delayedControl;

            uint64_t txInputSamples;
            uint64_t txZeroFillSamples;
            uint64_t rxSamples;
            uint64_t rxRssiClampSamples;
            uint64_t rxControlAlignedSamples;
            uint64_t rxControlDeferredSamples;
            size_t maxDelayedControlDepth;

            uint64_t droppedRxBytes;
            uint64_t droppedTxBytes;
        };

        /**
         * @brief Initializes a new instance of the RadioManager class.
         */
        RadioManager();
        /**
         * @brief Finalizes an instance of the RadioManager class.
         */
        ~RadioManager();

        RadioManager(const RadioManager&) = delete;
        RadioManager& operator=(const RadioManager&) = delete;

        /**
         * @brief Helper to ensure a channel state exists for a given modem ID. This method checks if a channel state already exists
         * for the specified modem ID, and if not, it creates a new channel state with default values and adds it to the 
         * channel map. This ensures that there is always a valid channel state for any modem ID that is accessed,
         * simplifying the logic for channel management and sample queuing.
         * @param modemId Modem ID for which to ensure a channel state exists.
         * @returns ChannelState& Reference to the channel state associated with the specified modem ID.
         */
        ChannelState& ensureChannel(uint8_t modemId);
        /**
         * @brief Helper to parse the SDR configuration from a YAML node. This method reads the SDR configuration 
         * parameters from the provided YAML node, initializes the device and channel states accordingly, and prepares 
         * the RadioManager for starting the GNU Radio flowgraphs. The configuration includes parameters for each SDR 
         * device, such as sample rate and gain, as well as the assignment of modem channels to specific devices and 
         * channel bindings.
         * @param conf YAML node containing the SDR configuration.
         * @returns bool True if the configuration was parsed successfully, otherwise false.
         */
        bool parseConfig(yaml::Node& conf);
        /**
         * @brief Helper to recompute the center frequencies for each SDR device based on the assigned channels. This method
         * iterates through all the channels assigned to each device, determines the minimum and maximum frequencies for 
         * both RX and TX, and calculates the center frequency as the average of the min and max. This is used to set 
         * the center frequency for the GNU Radio flowgraph, which can help optimize performance and reduce CPU load.
         */
        void recomputeDeviceCenters();
        /**
         * @brief Helper to start the GNU Radio flowgraphs for all configured SDR devices. This method creates and configures the
         * GNU Radio flowgraph for each device based on the device state and assigned channels, and starts the flowgraph to 
         * begin processing samples. It also sets up the IQ/sample shim blocks used by the channel queues.
         */
        void startRadios();
        /**
         * @brief Helper to stop the GNU Radio flowgraphs for all SDR devices. This method stops and cleans up the GNU Radio flowgraph
         * for each device, ensuring that all resources are released and the flowgraphs are properly shut down.
         */
        void stopRadios();
        /**
         * @brief Helper to apply retuning of the SDR devices based on any changes to the channel frequencies. This method is called when
         * channel parameters are updated, and it checks if the center frequencies for any devices need to be recalculated 
         * and applied to the GNU Radio flowgraphs. This allows for dynamic retuning of the SDR devices without needing to restart the flowgraphs.
         */
        void applyRetune();

        /**
         * @brief Helper to start the runtime status publisher thread. This method initializes and starts a background thread that periodically
         * publishes the runtime status of the RadioManager, including information about the devices, channels, and any 
         * diagnostics. The status is published to a configured address and topic, allowing for external monitoring of
         * the RadioManager's state.
         */
        void startRuntimeStatusPublisher();
        /**
         * @brief Helper to stop the runtime status publisher thread. This method signals the background thread responsible for publishing runtime status to stop,
         * and waits for the thread to finish before returning. This ensures a clean shutdown of the status publisher
         * thread and prevents any potential issues with dangling threads or resources.
         */
        void stopRuntimeStatusPublisher();
        /**
         * @brief Helper to publish the runtime status of the RadioManager. This method gathers information about the current
         * state of the devices, channels, and any diagnostics, and publishes this information to the configured address 
         * and topic. This allows for external monitoring of the RadioManager's state and can be useful for debugging 
         * and performance monitoring.
         */
        void publishRuntimeStatus();
        /**
         * @brief Helper to build a JSON string representing the current runtime status of the RadioManager. This method collects
         * information about the devices, channels, and diagnostics, and formats it into a JSON string that can be published
         * to the status topic. The JSON includes details such as device configurations, channel assignments, sample queue lengths, and any relevant diagnostics.
         * @returns std::string JSON string representing the current runtime status of the RadioManager.
         */
        std::string buildRuntimeStatusJson() const;

        /**
         * @brief Helper to log runtime diagnostics at regular intervals. This method checks if a certain amount of time has passed since the last diagnostics log,
         * and if so, it gathers diagnostic information about the devices and channels and logs it. This can include 
         * information such as sample queue lengths, dropped samples, and any errors or warnings. Regular logging of 
         * diagnostics can help identify performance issues or bottlenecks in the SDR operation.
         * @param nowMs Current time in milliseconds, used to determine if it's time to log diagnostics.
         */
        void logRuntimeDiagnostics(uint64_t nowMs);

        struct RuntimeContext;

        std::mutex m_lock;
        bool m_initialized;
        bool m_debug;

        std::unordered_map<uint8_t, ChannelState> m_channels;
        std::vector<DeviceState> m_devices;

        std::string m_runtimeStatusPubAddress;
        std::string m_runtimeStatusPubTopic;

        std::atomic<bool> m_statusThreadStop;
        std::thread m_statusThread;
        std::unique_ptr<RuntimeContext> m_runtime;
        uint64_t m_lastDiagnosticsLogMs;

#if defined(HAS_GNURADIO_ZEROMQ)
        void* m_zmqContext;
        void* m_zmqPubSocket;
#endif
    };
}

#endif // __RADIO_MANAGER_H__
