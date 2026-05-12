// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 * 
 *  Copyright (C) 2016 Jonathan Naylor, G4KLX
 *
 */
/**
 * @file CalRSSI.h
 * @ingroup modem_fw
 * @file CalRSSI.cpp
 * @ingroup modem_fw
 */
#if !defined(__CAL_RSSI_H__)
#define __CAL_RSSI_H__

#include "Defines.h"

namespace modem
{
    // ---------------------------------------------------------------------------
    //  Class Declaration
    // ---------------------------------------------------------------------------

    /**
     * @brief Implements logic for RSSI calibration mode.
     * @ingroup modem_fw
     */
    class DSP_FW_API CalRSSI {
    public:
        /**
         * @brief Initializes a new instance of the CalRSSI class.
             * @param modem The modem instance.
         */
        CalRSSI(modem::Modem* modem);

        /**
         * @brief Sample RSSI values from the air interface.
         * @param rssi 
         * @param length 
         */
        void samples(const uint16_t* rssi, uint8_t length);

    private:
        modem::Modem* m_modem;

        uint32_t m_count;
        uint32_t m_accum;
        uint16_t m_min;
        uint16_t m_max;
    };
} // namespace modem

#endif // __CAL_RSSI_H__
