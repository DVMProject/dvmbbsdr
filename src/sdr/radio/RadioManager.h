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
 * @brief Runtime SDR device and single-carrier bridge.
 *
 * This manager owns GNU Radio flowgraphs and provides a queue bridge between
 * virtual modem instances and SDR hardware channels.
 */
#if !defined(__RADIO_MANAGER_H__)
#define __RADIO_MANAGER_H__

#include "Defines.h"
#include "radio/FDUDC.h"
#include "common/yaml/Yaml.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <complex>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#if defined(HAS_SOAPYSDR)
namespace SoapySDR { class Device; class Stream; }
#endif

namespace radio
{
    /**
     * @brief Implements the RadioManager class, which manages a single active SDR carrier and bridges modem sample
     *  queues to SDR hardware. The runtime intentionally follows a single-carrier, MMDVM-IQ-style structure instead of
     *  wideband or multi-carrier channelization.
     */
    class DSP_FW_API RadioManager {
    public:
        /**
         * @brief Gets the singleton instance of the RadioManager. This method ensures that only one instance of the
         *  RadioManager exists throughout the application, providing a centralized point of access for managing SDR devices
         *  and channels. The instance is lazily initialized on the first call to this method, and subsequent calls will return the same instance.
         * @returns RadioManager& Reference to the singleton instance of the RadioManager.
         */
        static RadioManager& instance();

        /**
         * @brief Initializes the RadioManager with the given configuration and debug flag. This method sets up the SDR devices
         *  and channels based on the provided YAML configuration, and starts the runtime thread for managing SDR
         *  operation. The debug flag enables verbose logging for troubleshooting and development purposes. It is important to 
         *  call this method before using any other functionality of the RadioManager to ensure that the SDR environment is properly set up.
         * @param conf YAML node containing the configuration for the SDR devices and channels.
         * @param debug Boolean flag to enable or disable debug logging.
         * @returns bool True if initialization was successful, otherwise false.
         */
        bool initialize(yaml::Node& conf, bool debug);
        /**
         * @brief Shuts down the RadioManager, stopping all SDR devices and channels, and cleaning up resources.
         * This method should be called when the RadioManager is no longer needed to ensure a clean shutdown of the SDR runtime environment.
         */
        void shutdown();

        /**
         * @brief Helper to set the RF channel polarity for a specific modem channel. This method allows for dynamic reconfiguration
         *  of the channel's RX and TX polarity for RF path integration with different hardware chains.
         * @param modemId Modem ID of the channel to configure.
         * @param rxInvert True to invert the RX polarity, false for normal polarity.
         * @param txInvert True to invert the TX polarity, false for normal polarity.
         */
        void setChannelPolarity(uint8_t modemId, bool rxInvert, bool txInvert);
        /**
         * @brief Helper to set the RF channel parameters for a specific modem channel. This method allows for dynamic reconfiguration
         *  of the channel's frequencies, RF power level, and polarity.
         * @param modemId Modem ID of the active carrier.
         * @param rxFreq Receive frequency in Hz.
         * @param txFreq Transmit frequency in Hz.
         * @param rfPower RF power level hint.
         * @param rxInvert Boolean flag to invert the receive signal.
         * @param txInvert Boolean flag to invert the transmit signal.
         */
        void setChannelParams(uint8_t modemId, uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower, bool rxInvert, bool txInvert);
        /**
         * @brief Helper to configure AFC for a specific modem channel.
         * @param modemId Modem ID of the active carrier.
         * @param enable True to enable AFC.
         * @param afcKI AFC integral gain parameter.
         * @param afcKP AFC proportional gain parameter.
         * @param afcRange AFC correction range parameter.
         */
        void setChannelAFC(uint8_t modemId, bool enabled, uint8_t ki, uint8_t kp, uint8_t range);
        /**
         * @brief Helper to set the TX active state for a specific modem channel. This method is used to indicate whether 
         *  the channel is currently active for transmission, which affects whether samples from the channel's TX queue 
         *  are sent to the SDR hardware.
         * @param modemId Modem ID of the active carrier.
         * @param active True to activate TX, false to deactivate.
         */
        void setChannelTxActive(uint8_t modemId, bool active);

        /**
         * @brief Writes samples to the TX queue of a specific modem channel. This method is called by modem instances to queue
         *  samples for transmission. The samples and corresponding control data are stored in the channel's TX queue, and 
         *  will be sent to the SDR hardware when the channel is active. The method returns the number of samples
         *  that were actually written to the queue and will be transmitted.
         * @param modemId Modem ID of the active carrier.
         * @param samples Pointer to the array of samples to write.
         * @param control Pointer to the array of control data corresponding to the samples.
         * @param sampleCount Number of samples to write.
         * @returns size_t Number of samples actually written to the channel's TX queue.
         */
        size_t writeChannelTxSamples(uint8_t modemId, const int16_t* samples, const uint8_t* control, size_t sampleCount);
        /**
         * @brief Reads samples from the RX queue of a specific modem channel. This method is called by modem instances to retrieve
         *  samples that have been received from the SDR hardware. The method fills the provided pointers with the samples, 
         *  control data, and RSSI values from the channel's RX queue, up to a maximum defined by MAX_READ_SAMPLES. The 
         *  method returns the number of samples that were read from the queue and provided to the caller.
         * @param modemId Modem ID of the active carrier.
         * @param samples Reference to a pointer that will be set to point to the array of samples read from the 
         *  channel's RX queue.
         * @param control Reference to a pointer that will be set to point to the array of control data corresponding to the 
         *  samples read from the channel's RX queue.
         * @param rssi Reference to a pointer that will be set to point to the array of RSSI values corresponding to the 
         *  samples read from the channel's RX queue.
         * @returns int Number of samples actually read from the channel's RX queue.
         */
        int readChannelRxSamples(uint8_t modemId, int16_t*& samples, uint8_t*& control, uint16_t*& rssi);

    private:
        /**
         * @brief Structure to hold the state of a single SDR channel, including its configuration parameters, 
         *  sample queues, and runtime statistics. This structure is used internally by the RadioManager to manage the 
         *  state of each channel and facilitate the queuing of samples for transmission and reception.
         */
        struct DeviceState {
            std::string args;
            double sampleRate;
            double rxGain;
            double rxBandwidth;
            double txGain;
            double txBandwidth;
            double freqCorrPpm;
            double rxCenterOffsetHz;
            double txCenterOffsetHz;
            double rxCenterHz;
            double txCenterHz;
            std::string rxAntenna;
            std::string txAntenna;
            std::string clockSource;
            std::string timeSource;
            std::string rxGainElement;
            std::string txGainElement;

#if defined(HAS_SOAPYSDR)
            SoapySDR::Device* soapyDevice;
            SoapySDR::Stream* rxStream;
            SoapySDR::Stream* txStream;
            bool timestamped;
            bool streamActive;
            long long txTimeNs;
            long long txLatencyNs;
            long long lastRxTimeNs;
            bool lastRxTimeValid;
#endif

            uint32_t hwToModemDecim;
            uint32_t modemToHwInterp;

            uint64_t lastDiagLogMs;
            uint64_t clipSamples;
            float peakComposite;
            double minCarrierSpacingHz;
            double occupiedBandwidthHz;
            bool guardBandViolated;
        };

        /**
         * @brief Structure to hold the state of a single modem channel, including its assigned RX and TX devices,
         *  sample queues, and runtime statistics. This structure is used internally by the RadioManager to manage the 
         *  state of each modem channel and facilitate the queuing of samples for transmission and reception.
         */
        struct ChannelState {
            uint8_t modemId;
            size_t rxDevice;
            size_t txDevice;

            uint32_t rxFreq;
            uint32_t txFreq;
            uint8_t rfPower;

            bool rxInvert;
            bool txInvert;
            bool txActive;

            bool afcEnabled;
            uint8_t afcKI;
            uint8_t afcKP;
            uint8_t afcRange;

            double rxOffsetHz;
            double txOffsetHz;

            int32_t txPhaseWord;
            float txNcoPhase;
            float txNcoStep;

            std::complex<float> prevRx;
            float rxNcoPhase;
            float rxNcoStep;

            std::unique_ptr<FDUDC> fdudc;
            uint32_t fdudcNum;
            uint32_t fdudcDen;
            size_t controlDelaySamples;

            int16_t txHoldSample;
            uint8_t txHoldControl;
            uint32_t txInterpCounter;

            uint32_t rxDecimCounter;
            float rxDecimAcc;
            float rxRssiAcc;
            uint32_t rxRssiCount;

            std::deque<int16_t> txSamples;
            std::deque<uint8_t> txControl;
            std::deque<uint8_t> delayedControl;
            std::deque<int16_t> rxSamples;
            std::deque<uint8_t> rxControl;
            std::deque<uint16_t> rxRssi;

            std::vector<int16_t> readSamples;
            std::vector<uint8_t> readControl;
            std::vector<uint16_t> readRssi;

            std::mutex lock;
        };

        std::atomic<bool> m_running;
        bool m_debug;

        std::thread m_runtimeThread;
        std::mutex m_stateLock;

        std::vector<DeviceState> m_devices;
        std::unordered_map<uint8_t, size_t> m_modemRxDevice;
        std::unordered_map<uint8_t, size_t> m_modemTxDevice;
        std::map<uint8_t, ChannelState> m_channels;
        uint8_t m_primaryModemId;
        bool m_primaryModemValid;

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
         * @brief Parses the YAML configuration to set up the SDR devices and channels. This method reads the configuration
         *  parameters for each device and channel from the provided YAML node, and initializes the internal state of the 
         *  RadioManager accordingly. It also performs any necessary setup for the SDR devices based on the configuration, 
         *  such as initializing SoapySDR devices. This method is called during initialization of the RadioManager 
         *  to configure the SDR environment based on the provided configuration file.
         * @param conf YAML node containing the configuration for the SDR devices and channels.
         */
        void parseConfig(yaml::Node& conf);

        /**
         * @brief Recomputes the center frequencies for each SDR device based on the assigned channels and their 
         *  frequencies. This method is called whenever the channel configuration changes (e.g. frequencies, device 
         *  assignments) to ensure that the SDR devices are tuned to the appropriate center frequencies to accommodate 
         *  the assigned channels. It calculates the minimum and maximum frequencies for each device based on the 
         *  channels assigned to it, and sets the device center frequency to the midpoint of that range. This allows 
         *  for proper operation of the SDR devices and ensures that the channels are correctly centered within the 
         *  device's tuning range.
         */
        void recomputeDeviceCenters();
        /**
         * @brief Updates the channel frequency offsets based on the current device center frequencies. This method is called
         *  after recomputing the device centers to calculate the frequency offset for each channel relative to
         *  its assigned device's center frequency. This ensures that each channel operates at the correct frequency
         *  within the device's tuning range.
         */
        void updateChannelOffsets();
        /**
         * @brief Updates the channel spacing metrics for diagnostics and monitoring. This method calculates metrics 
         *  such as the minimum carrier spacing, occupied bandwidth, and guard band violations for the configured channels. 
         *  These metrics are used for diagnostics logging and can be published via ZeroMQ for real-time monitoring of the 
         *  SDR operation. This method should be called whenever the channel configuration changes to ensure that the 
         *  metrics are up to date.
         */
        void updateChannelSpacingMetrics();
        
        /**
         * @brief Main runtime loop for the RadioManager. This method runs in a separate thread and is responsible for 
         *  managing the operation of the SDR devices and channels, including processing sample queues, handling device 
         *  I/O, and performing diagnostics logging. The loop continues running until the RadioManager is shut down, at 
         *  which point it will cleanly exit and allow the thread to join. This method is the core of the RadioManager's 
         *  runtime behavior and should be designed to efficiently manage the SDR operation while minimizing latency 
         *  and ensuring timely processing of samples.
         */
        void runtimeLoop();

        /**
         * @brief Creates a default device state with standard parameters. This method is used to initialize the 
         *  RadioManager with a default device configuration in case no devices are specified in the configuration file. 
         *  The default device state includes standard parameters that are suitable for general use, and can be overridden 
         *  by the configuration file if specific parameters are provided.
         */
        DeviceState makeDefaultDevice() const;
        /**
         * @brief Retrieves a pointer to the ChannelState for a given modem ID. This method is used internally by the 
         *  RadioManager to access the state of a specific modem channel based on its modem ID. The returned pointer 
         *  can be used to read or modify the channel's state, including its configuration parameters, sample queues, 
         *  and runtime statistics.
         */
        ChannelState* getChannel(uint8_t modemId);

    #if defined(HAS_SOAPYSDR)
        /**
         * @brief Starts a SoapySDR device based on its index in the devices vector. This method initializes the SoapySDR
         *  device, sets up the RX and TX streams, and configures the device parameters based on the current configuration. 
         *  It also handles any necessary error checking and logging related to starting the device.
         */
        bool startSoapyDevice(size_t devIdx);
        /**
         * @brief Stops a SoapySDR device based on its index in the devices vector. This method deactivates the RX and 
         *  TX streams, releases any resources associated with the device, and performs any necessary cleanup. It also 
         *  handles error checking and logging related to stopping the device.
         */
        void stopSoapyDevice(size_t devIdx);
        /**
         * @brief Stops all SoapySDR devices that are currently active. This method iterates through all devices in the
         *  devices vector and calls stopSoapyDevice for each one that is active. This is typically called during 
         *  shutdown of the RadioManager to ensure that all SDR devices are cleanly stopped and resources are released.
         */
        void stopAllSoapyDevices();

        /**
         * @brief Reads samples from a SoapySDR device's RX stream. This method reads samples from the specified 
         *  device's RX stream and fills the provided buffer with the received IQ samples. It also handles any necessary 
         *  error checking and logging related to reading from the device.
         * @param devIdx Index of the device
         * @param iq Reference to a vector that will be filled with the complex float samples read from the device's RX stream.
         * @returns bool True if the samples were successfully read from the device, otherwise false.
         */
        bool readSoapyRx(size_t devIdx, std::vector<std::complex<float>>& iq);
        /**
         * @brief Writes samples to a SoapySDR device's TX stream. This method writes the provided IQ samples to the 
         *  specified device's TX stream for transmission. It also handles any necessary error checking and logging 
         *  related to writing to the device, and manages timing information for timestamped transmissions if applicable.
         * @param devIdx Index of the device
         * @param iq Vector of complex float samples to write to the device's TX stream.
         * @returns bool True if the samples were successfully written to the device, otherwise false.
         */
        bool writeSoapyTx(size_t devIdx, const std::vector<std::complex<float>>& iq);
    #endif
    };
}

#endif // __RADIO_MANAGER_H__
