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

#include <cstddef>
#include <cstdint>

namespace radio
{
    // ---------------------------------------------------------------------------
    //  Class Declaration
    // ---------------------------------------------------------------------------
    
    /**
     * @brief Singleton RF runtime manager for SDR and channel orchestration.
     * @ingroup modem_fw
     *
     * Responsibilities:
     *  - Parse SDR/device configuration.
     *  - Build and run GNU Radio flowgraphs.
     *  - Route per-modem RX/TX samples to channelized SDR paths.
     *  - Apply in-place hot retune operations for channel RF changes.
     */
    class HOST_SW_API RadioManager {
    public:
        /**
         * @brief Returns the process-global SDR runtime instance.
         */
        static RadioManager& instance();

        /**
         * @brief Initializes the RF runtime and starts SDR flowgraphs.
         * @param conf Root YAML configuration node.
         * @return true on success, false on failure.
         */
        bool initialize(yaml::Node& conf);
        /**
         * @brief Stops all flowgraphs and releases SDR resources.
         */
        void shutdown();

        /**
         * @brief Updates modem RF channel settings.
         *
         * This call applies hot retune updates without full graph rebuild when
         * channel topology does not change.
         *
         * @param modemId 1-based modem identifier.
         * @param rxFreq Receive frequency in Hz.
         * @param txFreq Transmit frequency in Hz.
         * @param rfPower RF power level hint.
         */
        void setChannelRF(uint8_t modemId, uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower);

        /**
         * @brief Queues modem-domain TX samples for SDR transmission.
         * @param modemId 1-based modem identifier.
         * @param samples Pointer to sample buffer.
         * @param length Buffer length in bytes.
         */
        void enqueueTx(uint8_t modemId, const uint8_t* samples, size_t length);
        /**
         * @brief Dequeues modem-domain RX samples from SDR path.
         * @param modemId 1-based modem identifier.
         * @param[out] samples Pointer to contiguous sample data owned by runtime.
         * @return Number of bytes available in samples.
         */
        int dequeueRx(uint8_t modemId, uint8_t*& samples);

    private:
        /**
         * @brief Initializes a new instance of the RadioManager class.
         */
        RadioManager();
        /**
         * @brief Finalizes a instance of the RadioManager class.
         */
        ~RadioManager();

        RadioManager(const RadioManager&) = delete;
        RadioManager& operator=(const RadioManager&) = delete;

        struct Impl;
        Impl* m_impl;
    };
}

#endif // __RADIO_MANAGER_H__
