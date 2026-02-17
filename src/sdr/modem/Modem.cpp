// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Modem Firmware
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2015,2016,2017 Jonathan Naylor, G4KLX
 *  Copyright (C) 2015 Jim Mclaughlin, KI6ZUM
 *  Copyright (C) 2016 Colin Durbridge, G4EML
 *  Copyright (C) 2017-2026 Bryan Biedenkapp, N2PLL
 *
 */
#include "modem/Modem.h"
#include "common/Log.h"

using namespace modem;

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the Modem class. */

Modem::Modem(port::IModemPort* port, bool verbose, bool debug) :
    m_port(port),
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
    m_io(this),
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
    ** TODO TODO TODO: this should probably be setting up whatever is needed
    **  on GNU Radio to have carrier frequency running
    */

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

    /*
    ** TODO TODO TODO: this should probably be shutdown whatever is needed
    **  on GNU Radio for a running carrier frequency
    */
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

/* Read samples queued for reception. */

int Modem::readFMSamples(uint8_t* samples)
{
    /*
    ** TODO TODO TODO -- this should pull demodulated FM samples from some queue for passing to 
    **  the higher level Rx functions
    */

    return 0;
}

/* Helper to handle FM modulated samples. */

void Modem::transmitFMSamples(const uint8_t* samples, size_t length)
{
    /*
    ** TODO TODO TODO -- this is fed samples from the higher level Tx functions
    **  to be transmitted as modulated FM
    */
}
