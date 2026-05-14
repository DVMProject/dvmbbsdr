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
     * @brief Enumeration of modulation modes supported by the SDR channels. This enum is used to specify the modulation
     * type for each channel, which determines how the samples are processed and transmitted by the GNU Radio flowgraph. 
     * The supported modulation modes include FM_C4FM for frequency modulation and IQ_CQPSK for complex IQ modulation.
     */
    enum class ModulationMode {
        FM_C4FM,
        IQ_CQPSK
    };

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
         * of the channel's RX and TX polarity, which can be necessary for certain modulation schemes or hardware requirements. 
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
         * @brief Helper to push samples to a channel's TX queue. This method is used by modem instances to queue 
         * samples for transmission on a specific channel. The samples will be sent to the SDR hardware when the channel 
         * is active for transmission.
         * @param modemId Modem ID of the channel to which the samples should be pushed.
         * @param samples Buffer containing the samples to push.
         * @param length Length of the samples buffer.
         */
        void pushChannelTxSamples(uint8_t modemId, const uint8_t* samples, size_t length);
        /**
         * @brief Helper to pop samples from a channel's RX queue. This method is used by modem instances to retrieve received
         * samples from a specific channel. The samples are removed from the channel's RX queue and returned to the caller for processing.
         * @param modemId Modem ID of the channel from which to pop samples.
         * @param samples Reference to a pointer that will be set to the buffer containing the popped samples.
         * @returns int Number of samples popped and stored in the provided buffer, or -1 if an error occurred (e.g., channel 
         * not found or no samples available in the queue).
         */
        int popChannelRxSamples(uint8_t modemId, uint8_t*& samples);

        /**
         * @brief Helper to dequeue samples from a channel's TX queue. This method is used internally by the GNU Radio flowgraph
         * to retrieve samples that have been queued for transmission by modem instances. The samples are removed from the 
         * channel's TX queue and returned to the caller for sending to the SDR hardware.
         * @param modemId Modem ID of the channel from which to dequeue samples.
         * @param dst Buffer to store the dequeued samples.
         * @param sampleCount Maximum number of samples to dequeue.
         * @returns size_t Number of samples dequeued and stored in the provided buffer, or 0 if no samples were available 
         * in the queue.
         */
        size_t dequeueChannelTxSamples(uint8_t modemId, int16_t* dst, size_t sampleCount);
        /**
         * @brief Helper to enqueue samples to a channel's RX queue. This method is used internally by the GNU Radio flowgraph
         * to store received samples from the SDR hardware into the appropriate channel's RX queue for processing by modem 
         * instances. The samples are added to the channel's RX queue and will be available for retrieval by modem instances.
         * @param modemId Modem ID of the channel to which the samples should be enqueued.
         * @param src Buffer containing the samples to enqueue.
         * @param sampleCount Number of samples in the buffer to enqueue.
         */
        void enqueueChannelRxSamples(uint8_t modemId, const int16_t* src, size_t sampleCount);

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
         * @brief Structure to hold the state of a single modem channel, including its assigned RX and TX devices, modulation mode,
         * sample queues, and runtime statistics. This structure is used internally by the RadioManager to manage the 
         * state of each modem channel and facilitate the queuing of samples for transmission and reception.
         */
        struct ChannelState {
            uint8_t modemId;
            int rxDevice;
            int txDevice;
            ModulationMode modulation;

            uint32_t rxFreq;
            uint32_t txFreq;
            uint8_t rfPower;
            bool rxInvert;
            bool txInvert;
            bool txActive;

            std::deque<uint8_t> rxQueue;
            std::deque<uint8_t> txQueue;
            std::vector<uint8_t> rxScratch;

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
         * modulation modes.
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
         * begin processing samples. It also sets up any necessary blocks for handling the sample queues and modulation processing.
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
