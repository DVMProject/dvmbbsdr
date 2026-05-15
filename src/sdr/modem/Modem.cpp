// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 *
 */
#include "modem/Modem.h"
#include "common/Thread.h"
#include "common/StopWatch.h"
#include "common/Log.h"
#include "radio/RadioManager.h"
#include "SDRMain.h"

using namespace modem;

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the Modem class. */

Modem::Modem(port::IModemPort* port, uint8_t id, std::string ptyPort, bool verbose, bool debug) :
    m_port(port),
    m_modemId(id + 1),
    m_modemPty(ptyPort),
    m_modemState(STATE_IDLE),
    m_dmrEnable(true),
    m_p25Enable(true),
    m_nxdnEnable(true),
    m_dcBlockerEnable(true),
    m_cosLockoutEnable(false),
    m_duplex(true),
    m_tx(false),
    m_dcd(false),
    m_serial(this),
    m_io(this, debug),
    m_verbose(verbose),
    m_debug(debug),
    m_dmrIdleRX(this),
    m_dmrRX(this),
    m_dmrTX(this),
    m_dmrDMORX(this),
    m_dmrDMOTX(this),
    m_p25RX(this),
    m_p25TX(this),
    m_nxdnRX(this),
    m_nxdnTX(this),
    m_calDMR(this),
    m_calP25(this),
    m_calNXDN(this),
    m_calRSSI(this),
    m_cwIdTX(this)
{
    /* stub */
}

/* Finalizes a instance of the Modem class. */

Modem::~Modem()
{
    /* stub */
}

/* Opens connection to the air interface modem. */

bool Modem::open()
{
    bool ret = m_port->open();
    if (!ret)
        return false;

    /*
    ** Initialize Threads
    */

    if (!Thread::runAsThread(this, threadClock))
        return EXIT_FAILURE;

    return true;
}

/* Process samples from air interface. */

void Modem::clock(uint32_t ms)
{
    m_serial.process();
    m_io.process();

    // The following is for transmitting
    if (m_dmrEnable && m_modemState == STATE_DMR) {
        if (m_duplex)
            m_dmrTX.process();
        else
            m_dmrDMOTX.process();
    }

    if (m_p25Enable && m_modemState == STATE_P25)
        m_p25TX.process();

    if (m_nxdnEnable && m_modemState == STATE_NXDN)
        m_nxdnTX.process();

    if (m_modemState == STATE_DMR_DMO_CAL_1K || m_modemState == STATE_DMR_CAL_1K ||
        m_modemState == STATE_DMR_LF_CAL || m_modemState == STATE_DMR_CAL)
        m_calDMR.process();

    if (m_modemState == STATE_P25_CAL_1K || m_modemState == STATE_P25_CAL)
        m_calP25.process();

    if (m_modemState == STATE_NXDN_CAL)
        m_calNXDN.process();

    if (m_modemState == STATE_CW || m_modemState == STATE_IDLE)
        m_cwIdTX.process();
}

/* Closes connection to the air interface modem. */

void Modem::close()
{
    m_port->close();
}

/* */

void Modem::writeDebug(const char* text)
{
    m_serial.writeDebug(text);
}

/* */

void Modem::writeDebug(const char* text, int16_t n1)
{
    m_serial.writeDebug(text, n1);
}

/* */

void Modem::writeDebug(const char* text, int16_t n1, int16_t n2)
{
    m_serial.writeDebug(text, n1, n2);
}

/* */

void Modem::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3)
{
    m_serial.writeDebug(text, n1, n2, n3);
}

/* */

void Modem::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4)
{
    m_serial.writeDebug(text, n1, n2, n3, n4);
}

/* */

void Modem::writeDump(const uint8_t* data, uint16_t length)
{
    m_serial.writeDump(data, length);
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Entry point to clock thread. */

void* Modem::threadClock(void* arg)
{
    thread_t* th = (thread_t*)arg;
    if (th != nullptr) {
#if defined(_WIN32)
        ::CloseHandle(th->thread);
#else
        ::pthread_detach(th->thread);
#endif // defined(_WIN32)

        std::string threadName("modem:clock");
        Modem* modem = static_cast<Modem*>(th->obj);
        if (modem == nullptr) {
            g_killed = true;
            LogError(LOG_HOST, "[FAIL] %s", threadName.c_str());
        } else {
            threadName = "modem:clock:" + std::to_string(modem->m_modemId);
        }

        if (g_killed) {
            delete th;
            return nullptr;
        }

        LogInfoEx(LOG_HOST, "[ OK ] %s (%s)", threadName.c_str(), modem->m_modemPty.c_str());
#ifdef _GNU_SOURCE
        ::pthread_setname_np(th->thread, threadName.c_str());
#endif // _GNU_SOURCE

        StopWatch stopWatch;
        stopWatch.start();

        while (!g_killed) {
            uint32_t ms = stopWatch.elapsed();
            stopWatch.start();

            modem->clock(ms);

            if (ms <= 1U)
                Thread::sleep(1U);
        }

        LogInfoEx(LOG_HOST, "[STOP] %s", threadName.c_str());
        delete th;
    }

    return nullptr;
}

/* Helper to set RF channel parameters for this modem instance. */

void Modem::setRFChannel(uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower, bool rxInvert, bool txInvert)
{
    radio::RadioManager::instance().setChannelPolarity(m_modemId, rxInvert, txInvert);
    radio::RadioManager::instance().setChannelParams(m_modemId, rxFreq, txFreq, rfPower);
}

/* Helper to set the modem TX activity state. */

void Modem::setModemTxActive(bool active)
{
    radio::RadioManager::instance().setChannelTxActive(m_modemId, active);
}

/* Read samples queued for reception. */

int Modem::readRFSamples(int16_t*& samples, uint8_t*& control, uint16_t*& rssi)
{
    return radio::RadioManager::instance().popChannelRxSamples(m_modemId, samples, control, rssi);
}

/* Queue RF samples for IQ modulation/transmit. */

void Modem::transmitRFSamples(const int16_t* samples, const uint8_t* control, size_t sampleCount)
{
    radio::RadioManager::instance().pushChannelTxSamples(m_modemId, samples, control, sampleCount);
}