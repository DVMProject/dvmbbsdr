// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 *
 */
/**
 * @file Modem.h
 * @ingroup modem_fw
 * @file Modem.cpp
 * @ingroup modem_fw
 */
#if !defined(__MODEM_H__)
#define __MODEM_H__

#include "Defines.h"

#include "dmr/DMRIdleRX.h"
#include "dmr/DMRDMORX.h"
#include "dmr/DMRDMOTX.h"
#include "dmr/DMRRX.h"
#include "dmr/DMRTX.h"
#include "dmr/CalDMR.h"

#include "p25/P25RX.h"
#include "p25/P25TX.h"
#include "p25/CalP25.h"

#include "nxdn/NXDNRX.h"
#include "nxdn/NXDNTX.h"
#include "nxdn/CalNXDN.h"

#include "modem/CalRSSI.h"
#include "modem/CWIdTX.h"

#include "modem/SerialPort.h"
#include "modem/IO.h"

#include "modem/port/IModemPort.h"

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

const uint8_t   MARK_SLOT1 = 0x08U;
const uint8_t   MARK_SLOT2 = 0x04U;
const uint8_t   MARK_NONE = 0x00U;

const uint16_t  RX_BLOCK_SIZE = 2U;

const uint16_t  TX_RINGBUFFER_SIZE = 500U;
const uint16_t  RX_RINGBUFFER_SIZE = 600U;

namespace modem
{
    // ---------------------------------------------------------------------------
    //  Class Declaration
    // ---------------------------------------------------------------------------

    /**
     * @brief Implements the core interface to the modem hardware.
     * @ingroup modem_fw
     */
    class DSP_FW_API Modem {
    public:
        /**
         * @brief Initializes a new instance of the Modem class.
         * @param port The PTY port for this virtual modem.
         * @param id Numerical identifier for this virtual modem.
         * @param ptyPort String for the PTY port for this modem.
         * @param verbose Flag indicating whether air interface modem verbose is enabled.
         * @param debug Flag indicating whether air interface modem debug is enabled.
         */
        Modem(port::IModemPort* port, uint8_t id, std::string ptyPort, bool verbose, bool debug);
        /**
         * @brief Finalizes a new instance of the Modem class.
         */
        ~Modem();

        /**
         * @brief Opens connection to the air interface modem.
         * @returns bool True, if connection to modem is made, otherwise false.
         */
        bool open();

        /**
         * @brief Updates the modem by the passed number of milliseconds.
         *  NOTE: This shouldn't be called directly, this will be handled by the internal thread.
         * @param ms Number of milliseconds.
         */
        void clock(uint32_t ms);

        /**
         * @brief Closes connection to the air interface modem.
         */
        void close();

        /**
         * @brief 
         * @param[in] text
         */
        void writeDebug(const char* text);
        /**
         * @brief 
         * @param[in] text
         * @param n1 
         */
        void writeDebug(const char* text, int16_t n1);
        /**
         * @brief 
         * @param[in] text
         * @param n1 
         * @param n2 
         */
        void writeDebug(const char* text, int16_t n1, int16_t n2);
        /**
         * @brief 
         * @param[in] text
         * @param n1 
         * @param n2 
         * @param n3
         */
        void writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3);
        /**
         * @brief 
         * @param[in] text
         * @param n1 
         * @param n2 
         * @param n3
         * @param n4 
         */
        void writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4);
        /**
         * @brief 
         * @param[in] data 
         * @param length
         */
        void writeDump(const uint8_t* data, uint16_t length);

    private:
        friend class modem::SerialPort;
        friend class modem::IO;

        port::IModemPort* m_port;

        uint8_t m_modemId;
        std::string m_modemPty;

        DVM_STATE m_modemState;

        bool m_dmrEnable;
        bool m_p25Enable;
        bool m_nxdnEnable;

        bool m_dcBlockerEnable;
        bool m_cosLockoutEnable;

        bool m_duplex;

        bool m_tx;
        bool m_dcd;

        SerialPort m_serial;
        IO m_io;

        bool m_verbose;
        bool m_debug;

        /* DMR BS */
        friend class dmr::DMRIdleRX;
        friend class dmr::DMRRX;
        friend class dmr::DMRSlotRX;
        friend class dmr::DMRTX;
        dmr::DMRIdleRX m_dmrIdleRX;
        dmr::DMRRX m_dmrRX;
        dmr::DMRTX m_dmrTX;

        /* DMR MS-DMO */
        friend class dmr::DMRDMORX;
        friend class dmr::DMRDMOTX;
        dmr::DMRDMORX m_dmrDMORX;
        dmr::DMRDMOTX m_dmrDMOTX;

        /* P25 */
        friend class p25::P25RX;
        friend class p25::P25TX;
        p25::P25RX m_p25RX;
        p25::P25TX m_p25TX;

        /* NXDN */
        friend class nxdn::NXDNRX;
        friend class nxdn::NXDNTX;
        nxdn::NXDNRX m_nxdnRX;
        nxdn::NXDNTX m_nxdnTX;

        /* Calibration */
        friend class dmr::CalDMR;
        friend class p25::CalP25;
        friend class nxdn::CalNXDN;
        friend class modem::CalRSSI;
        dmr::CalDMR m_calDMR;
        p25::CalP25 m_calP25;
        nxdn::CalNXDN m_calNXDN;
        CalRSSI m_calRSSI;

        /* CW */
        friend class modem::CWIdTX;
        CWIdTX m_cwIdTX;

        /**
         * @brief Entry point to clock thread.
         * @param arg Instance of the thread_t structure.
         * @returns void* (Ignore)
         */
        static void* threadClock(void* arg);

        /**
         * @brief Helper to set RF channel parameters for this modem instance.
         * @param rxFreq Receive frequency in Hz.
         * @param txFreq Transmit frequency in Hz.
         * @param rfPower RF power level hint.
         * @param rxInvert Flag indicating whether Rx polarity should be inverted.
         * @param txInvert Flag indicating whether Tx polarity should be inverted.
         */
        void setRFChannel(uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower, bool rxInvert, bool txInvert);

        /**
         * @brief Helper to set the modem TX activity state.
         * @param active True while modem TX is asserted and modulation should pass to SDR.
         */
        void setModemTxActive(bool active);

        /**
         * @brief Read samples queued for reception.
         * @param samples Buffer to store the received samples.
         * @return int Number of samples read.
         */
        int readFMSamples(uint8_t* samples);
        /**
         * @brief Helper to handle transmitting FM modulated samples.
         * @param samples Buffer containing samples.
         * @param length Length of buffer.
         */
        void transmitFMSamples(const uint8_t* samples, size_t length);
    };
} // namespace modem

#endif // __MODEM_H__
