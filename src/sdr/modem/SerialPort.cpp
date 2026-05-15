// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2013,2015,2016,2017 Jonathan Naylor, G4KLX
 *  Copyright (C) 2016 Colin Durbridge, G4EML
 *  Copyright (C) 2017-2024 Bryan Biedenkapp, N2PLL
 *
 */
#include "modem/SerialPort.h"
#include "modem/port/UARTPort.h"
#include "modem/Modem.h"
#include "common/Log.h"
#include "common/Utils.h"

using namespace modem;

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

#define concat(a, b, c) a " (build " b " " c ")"
const char HARDWARE[] = concat(DESCRIPTION, __TIME__, __DATE__);

const uint8_t PROTOCOL_VERSION = 4U;

// ---------------------------------------------------------------------------
//  Globals Variables
// ---------------------------------------------------------------------------

static uint8_t s_readBuffer = 0x00U;

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the SerialPort class. */

SerialPort::SerialPort(modem::Modem* modem) :
    m_modem(modem),
    m_buffer(),
    m_ptr(0U),
    m_len(0U),
    m_dblFrame(false),
    m_debug(false),
    m_repeat()
{
    // stub
}

/* Starts serial port communications. */

void SerialPort::start()
{
    beginInt(1U, port::SERIAL_115200);
}

/* Process data from serial port. */

void SerialPort::process()
{
    while (availableInt(1U)) {
        uint8_t c = readInt(1U);

        if (m_ptr == 0U) {
            if (c == DVM_SHORT_FRAME_START) {
                // Handle the frame start correctly
                m_buffer[0U] = c;
                m_ptr = 1U;
                m_len = 0U;
                m_dblFrame = false;
            }
            else if (c == DVM_LONG_FRAME_START) {
                // Handle the frame start correctly
                m_buffer[0U] = c;
                m_ptr = 1U;
                m_len = 0U;
                m_dblFrame = true;
            }
            else {
                m_ptr = 0U;
                m_len = 0U;
                m_dblFrame = false;
            }
        }
        else if (m_ptr == 1U) {
            // Handle the frame length
            if (m_dblFrame) {
                m_buffer[m_ptr] = c;
                m_len = ((c & 0xFFU) << 8);
                // DEBUG3("long frame, len msb", m_len, c);
            } else {
                m_len = m_buffer[m_ptr] = c;
                // DEBUG2("short frame, len", m_len);
            }
            m_ptr = 2U;
        }
        else if (m_ptr == 2U && m_dblFrame) {
            // Handle the frame length
            m_buffer[m_ptr] = c;
            m_len = (m_len + (c & 0xFFU));
            if (m_len > SERIAL_FB_LEN)
                m_len = SERIAL_FB_LEN; // don't allow length to be longer then the buffer
            // DEBUG3("long frame, len lsb", m_len, c);
            m_ptr = 3U;
        }
        else {
            // Any other bytes are added to the buffer
            m_buffer[m_ptr] = c;
            m_ptr++;

            // The full packet has been received, process it
            if (m_ptr == m_len) {
                uint8_t err = 2U;
                uint8_t offset = 2U;
                if (m_dblFrame)
                    offset = 3U;

                // DEBUG4("m_buffer [b0 - b2]", m_buffer[0], m_buffer[1], m_buffer[2]);
                // DEBUG4("m_buffer [b3 - b5]", m_buffer[3], m_buffer[4], m_buffer[5]);

                switch (m_buffer[offset]) {
                case CMD_GET_STATUS:
                    getStatus();
                    break;

                case CMD_GET_VERSION:
                    getVersion();
                    break;

                case CMD_SET_CONFIG:
                    err = setConfig(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case CMD_SET_MODE:
                    err = setMode(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case CMD_SET_SYMLVLADJ:
                    err = setSymbolLvlAdj(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case CMD_SET_RXLEVEL:
                    err = setRXLevel(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case CMD_SET_RFPARAMS:
                    err = setRFParams(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case CMD_CAL_DATA:
                    if (m_modem->m_modemState == STATE_DMR_DMO_CAL_1K || m_modem->m_modemState == STATE_DMR_CAL_1K ||
                        m_modem->m_modemState == STATE_DMR_LF_CAL || m_modem->m_modemState == STATE_DMR_CAL)
                        err = m_modem->m_calDMR.write(m_buffer + 3U, m_len - 3U);
                    if (m_modem->m_modemState == STATE_P25_CAL_1K || m_modem->m_modemState == STATE_P25_CAL)
                        err = m_modem->m_calP25.write(m_buffer + 3U, m_len - 3U);
                    if (m_modem->m_modemState == STATE_NXDN_CAL)
                        err = m_modem->m_calNXDN.write(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK) {
                        sendACK();
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid calibration data, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid calibration data", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_FLSH_READ:
                    flashRead();
                    break;

                case CMD_FLSH_WRITE:
                    err = flashWrite(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK) {
                        sendACK();
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid data to write to flash, err = %d", err);
                        m_modem->writeDebug("SerialPort::process() received invalid data to write to flash", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_RESET_MCU:
                    m_modem->m_io.resetMCU();
                    break;

                case CMD_SET_BUFFERS:
                    err = setBuffers(m_buffer + 3U, m_len - 3U);
                    if (err == RSN_OK) {
                        sendACK();
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid data to set buffers, err = %d", err);
                        m_modem->writeDebug("SerialPort::process() received invalid data to set buffers", err);
                        sendNAK(err);
                    }
                    break;

                /** CW */
                case CMD_SEND_CWID:
                    err = RSN_RINGBUFF_FULL;
                    if (m_modem->m_modemState == STATE_IDLE)
                        err = m_modem->m_cwIdTX.write(m_buffer + 3U, m_len - 3U);
                    if (err != RSN_OK) {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid CW Id data, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() invalid CW Id data", err);
                        sendNAK(err);
                    }
                    break;

                /** Digital Mobile Radio */
                case CMD_DMR_DATA1:
                    if (m_modem->m_dmrEnable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_DMR) {
                            if (m_modem->m_duplex)
                                err = m_modem->m_dmrTX.writeData1(m_buffer + 3U, m_len - 3U);
                        }
                    }
                    if (err == RSN_OK) {
                        if (m_modem->m_modemState == STATE_IDLE)
                            setMode(STATE_DMR);
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid DMR data, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort: process() received invalid DMR data", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_DMR_DATA2:
                    if (m_modem->m_dmrEnable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_DMR) {
                            if (m_modem->m_duplex)
                                err = m_modem->m_dmrTX.writeData2(m_buffer + 3U, m_len - 3U);
                            else
                                err = m_modem->m_dmrDMOTX.writeData(m_buffer + 3U, m_len - 3U);
                        }
                    }
                    if (err == RSN_OK) {
                        if (m_modem->m_modemState == STATE_IDLE)
                            setMode(STATE_DMR);
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid DMR data, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid DMR data", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_DMR_START:
                    if (m_modem->m_dmrEnable) {
                        err = RSN_INVALID_DMR_START;
                        if (m_len == 4U) {
                            if (m_buffer[3U] == 0x01U && m_modem->m_modemState == STATE_DMR) {
                                if (!m_modem->m_tx)
                                    m_modem->m_dmrTX.setStart(true);
                                err = RSN_OK;
                            }
                            else if (m_buffer[3U] == 0x00U && m_modem->m_modemState == STATE_DMR) {
                                if (m_modem->m_tx)
                                    m_modem->m_dmrTX.setStart(false);
                                err = RSN_OK;
                            }
                        }
                    }
                    if (err != RSN_OK) {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid DMR start, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid DMR start", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_DMR_SHORTLC:
                    if (m_modem->m_dmrEnable)
                        err = m_modem->m_dmrTX.writeShortLC(m_buffer + 3U, m_len - 3U);
                    if (err != RSN_OK) {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid DMR Short LC, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid DMR Short LC", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_DMR_ABORT:
                    if (m_modem->m_dmrEnable)
                        err = m_modem->m_dmrTX.writeAbort(m_buffer + 3U, m_len - 3U);
                    if (err != RSN_OK) {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid DMR Abort, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid DMR Abort", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_DMR_CACH_AT_CTRL:
                    if (m_modem->m_dmrEnable) {
                        err = RSN_INVALID_REQUEST;
                        if (m_len == 4U) {
                            m_modem->m_dmrTX.setIgnoreCACH_AT(m_buffer[3U]);
                            err = RSN_OK;
                        }
                    }
                    if (err != RSN_OK) {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid DMR CACH AT Control, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid DMR CACH AT Control", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_DMR_CLEAR1:
                    if (m_modem->m_dmrEnable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_P25)
                            m_modem->m_dmrTX.resetFifo1();
                    }
                    break;
                case CMD_DMR_CLEAR2:
                    if (m_modem->m_dmrEnable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_P25)
                            m_modem->m_dmrTX.resetFifo2();
                    }
                    break;

                /** Project 25 */
                case CMD_P25_DATA:
                    if (m_modem->m_p25Enable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_P25) {
                            if (m_dblFrame)
                                m_modem->m_p25TX.writeData(m_buffer + 4U, m_len - 4U);
                            else
                                err = m_modem->m_p25TX.writeData(m_buffer + 3U, m_len - 3U);
                        }
                    }
                    if (err == RSN_OK) {
                        if (m_modem->m_modemState == STATE_IDLE)
                            setMode(STATE_P25);
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid P25 data, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid P25 data", err);
                        sendNAK(err);
                    }
                    break;

                case CMD_P25_CLEAR:
                    if (m_modem->m_p25Enable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_P25)
                            m_modem->m_p25TX.clear();
                    }
                    break;

                /** Next Generation Digital Narrowband */
                case CMD_NXDN_DATA:
                    if (m_modem->m_nxdnEnable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_NXDN)
                            err = m_modem->m_nxdnTX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == RSN_OK) {
                        if (m_modem->m_modemState == STATE_IDLE)
                            setMode(STATE_NXDN);
                    }
                    else {
                        ::LogError(LOG_SERIAL, "SerialPort::process() received invalid NXDN data, modemState = %u, err = %d", m_modem->m_modemState, err);
                        m_modem->writeDebug("SerialPort::process() received invalid NXDN data", err);
                        sendNAK(err);
                    }
                    break;
                case CMD_NXDN_CLEAR:
                    if (m_modem->m_nxdnEnable) {
                        if (m_modem->m_modemState == STATE_IDLE || m_modem->m_modemState == STATE_P25)
                            m_modem->m_nxdnTX.clear();
                    }
                    break;

                default:
                    // Handle this, send a NAK back
                    sendNAK(RSN_NAK);
                    break;
                }

                m_ptr = 0U;
                m_len = 0U;
                m_dblFrame = false;
            }
        }
    }

    if (m_modem->m_io.getWatchdog() >= 48000U) {
        m_ptr = 0U;
        m_len = 0U;
        m_dblFrame = false;
    }
}

/* Helper to check if the modem is in a calibration state. */

bool SerialPort::isCalState(DVM_STATE state)
{
    // calibration mode check
    if (state == STATE_P25_CAL_1K ||
        state == STATE_DMR_DMO_CAL_1K || state == STATE_DMR_CAL_1K ||
        state == STATE_DMR_LF_CAL ||
        state == STATE_RSSI_CAL ||
        state == STATE_P25_CAL || state == STATE_DMR_CAL || state == STATE_NXDN_CAL) {
        return true;
    }

    return false;
}

/* Helper to determine digital mode if the modem is in a calibration state. */

DVM_STATE SerialPort::calRelativeState(DVM_STATE state)
{
    if (isCalState(state)) {
        if (state == STATE_DMR_DMO_CAL_1K || state == STATE_DMR_CAL_1K ||
            state == STATE_DMR_LF_CAL || state == STATE_DMR_CAL ||
            state == STATE_RSSI_CAL) {
            return STATE_DMR;
        } else if (state == STATE_P25_CAL_1K ||
            state == STATE_P25_CAL) {
            return STATE_P25;
        } else if (state == STATE_NXDN_CAL) {
            return STATE_NXDN;
        }
    }

    return STATE_CW;
}

/* Write DMR frame data to serial port. */

void SerialPort::writeDMRData(bool slot, const uint8_t* data, uint8_t length)
{
    if (m_modem->m_modemState != STATE_DMR && m_modem->m_modemState != STATE_IDLE)
        return;

    if (!m_modem->m_dmrEnable)
        return;

    if (length + 3U > 40U) {
        m_buffer[2U] = slot ? CMD_DMR_DATA2 : CMD_DMR_DATA1;
        sendNAK(RSN_ILLEGAL_LENGTH);
        return;
    }

    uint8_t reply[40U];
    ::memset(reply, 0x00U, 40U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = length + 3U;
    reply[2U] = slot ? CMD_DMR_DATA2 : CMD_DMR_DATA1;

    ::memcpy(reply + 3U, data, length);

    writeInt(1U, reply, length + 3U);
}

/* Write lost DMR frame data to serial port. */

void SerialPort::writeDMRLost(bool slot)
{
    if (m_modem->m_modemState != STATE_DMR && m_modem->m_modemState != STATE_IDLE)
        return;

    if (!m_modem->m_dmrEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = slot ? CMD_DMR_LOST2 : CMD_DMR_LOST1;

    writeInt(1U, reply, 3);
}

/* Write P25 frame data to serial port. */

void SerialPort::writeP25Data(const uint8_t* data, uint16_t length)
{
    if (m_modem->m_modemState != STATE_P25 && m_modem->m_modemState != STATE_IDLE)
        return;

    if (!m_modem->m_p25Enable)
        return;

    if (length + 4U > 520U) {
        m_buffer[2U] = CMD_P25_DATA;
        sendNAK(RSN_ILLEGAL_LENGTH);
        return;
    }

    uint8_t reply[520U];
    ::memset(reply, 0x00U, 520U);

    if (length < 252U) {
        reply[0U] = DVM_SHORT_FRAME_START;
        reply[1U] = length + 3U;
        reply[2U] = CMD_P25_DATA;
        ::memcpy(reply + 3U, data, length);

        writeInt(1U, reply, length + 3U);
    }
    else {
        length += 4U;
        reply[0U] = DVM_LONG_FRAME_START;
        reply[1U] = (length >> 8U) & 0xFFU;
        reply[2U] = (length & 0xFFU);
        reply[3U] = CMD_P25_DATA;
        ::memcpy(reply + 4U, data, length);

        writeInt(1U, reply, length + 4U);
    }
}

/* Write lost P25 frame data to serial port. */

void SerialPort::writeP25Lost()
{
    if (m_modem->m_modemState != STATE_P25 && m_modem->m_modemState != STATE_IDLE)
        return;

    if (!m_modem->m_p25Enable)
        return;

    uint8_t reply[3U];

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = CMD_P25_LOST;

    writeInt(1U, reply, 3);
}

/* Write NXDN frame data to serial port. */

void SerialPort::writeNXDNData(const uint8_t* data, uint8_t length)
{
    if (m_modem->m_modemState != STATE_NXDN && m_modem->m_modemState != STATE_IDLE)
        return;

    if (!m_modem->m_nxdnEnable)
        return;

    if (length + 3U > 130U) {
        m_buffer[2U] = CMD_NXDN_DATA;
        sendNAK(RSN_ILLEGAL_LENGTH);
        return;
    }

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = length + 3U;
    reply[2U] = CMD_NXDN_DATA;

    ::memcpy(reply + 3U, data, length);

    writeInt(1U, reply, length + 3U);
}

/* Write lost NXDN frame data to serial port. */

void SerialPort::writeNXDNLost()
{
    if (m_modem->m_modemState != STATE_NXDN && m_modem->m_modemState != STATE_IDLE)
        return;

    if (!m_modem->m_nxdnEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = CMD_NXDN_LOST;

    writeInt(1U, reply, 3);
}

/* Write calibration frame data to serial port. */

void SerialPort::writeCalData(const uint8_t* data, uint8_t length)
{
    if (length + 3U > 130U) {
        m_buffer[2U] = CMD_CAL_DATA;
        sendNAK(RSN_ILLEGAL_LENGTH);
        return;
    }

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = length + 3U;
    reply[2U] = CMD_CAL_DATA;

    ::memcpy(reply + 3U, data, length);

    writeInt(1U, reply, length + 3U);
}

/* Write RSSI frame data to serial port. */

void SerialPort::writeRSSIData(const uint8_t* data, uint8_t length)
{
    if (m_modem->m_modemState != STATE_RSSI_CAL)
        return;

    if (length + 3U > 130U) {
        m_buffer[2U] = CMD_RSSI_DATA;
        sendNAK(RSN_ILLEGAL_LENGTH);
        return;
    }

    uint8_t reply[30U];
    ::memset(reply, 0x00U, 30U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = length + 3U;
    reply[2U] = CMD_RSSI_DATA;

    ::memcpy(reply + 3U, data, length);

    writeInt(1U, reply, length + 3U);
}

/* */

void SerialPort::writeDebug(const char* text)
{
    if (!m_debug)
        return;

    if (m_modem->m_debug)
        ::LogDebug(LOG_SERIAL, "DSP_FW_API %s", text);

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = CMD_DEBUG1;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[1U] = count;

    writeInt(1U, reply, count, true);
}

/* */

void SerialPort::writeDebug(const char* text, int16_t n1)
{
    if (!m_debug)
        return;

    if (m_modem->m_debug)
        ::LogDebug(LOG_SERIAL, "DSP_FW_API %s %X", text, n1);

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = CMD_DEBUG2;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[1U] = count;

    writeInt(1U, reply, count, true);
}

/* */

void SerialPort::writeDebug(const char* text, int16_t n1, int16_t n2)
{
    if (!m_debug)
        return;

    if (m_modem->m_debug)
        ::LogDebug(LOG_SERIAL, "DSP_FW_API %s %X %X", text, n1, n2);

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = CMD_DEBUG3;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[count++] = (n2 >> 8) & 0xFF;
    reply[count++] = (n2 >> 0) & 0xFF;

    reply[1U] = count;

    writeInt(1U, reply, count, true);
}

/* */

void SerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3)
{
    if (!m_debug)
        return;

    if (m_modem->m_debug)
        ::LogDebug(LOG_SERIAL, "DSP_FW_API %s %X %X %X", text, n1, n2, n3);

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = CMD_DEBUG4;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[count++] = (n2 >> 8) & 0xFF;
    reply[count++] = (n2 >> 0) & 0xFF;

    reply[count++] = (n3 >> 8) & 0xFF;
    reply[count++] = (n3 >> 0) & 0xFF;

    reply[1U] = count;

    writeInt(1U, reply, count, true);
}

/* */

void SerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4)
{
    if (!m_debug)
        return;

    if (m_modem->m_debug)
        ::LogDebug(LOG_SERIAL, "DSP_FW_API %s %X %X %X %X", text, n1, n2, n3, n4);

    uint8_t reply[130U];
    ::memset(reply, 0x00U, 130U);

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = CMD_DEBUG5;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[count++] = (n2 >> 8) & 0xFF;
    reply[count++] = (n2 >> 0) & 0xFF;

    reply[count++] = (n3 >> 8) & 0xFF;
    reply[count++] = (n3 >> 0) & 0xFF;

    reply[count++] = (n4 >> 8) & 0xFF;
    reply[count++] = (n4 >> 0) & 0xFF;

    reply[1U] = count;

    writeInt(1U, reply, count, true);
}

/* */

void SerialPort::writeDump(const uint8_t* data, uint16_t length)
{
    if (!m_debug)
        return;

    if (m_modem->m_debug)
        Utils::dump(1U, "DSP_FW_API Dump", data, length);

    if (length + 4U > 516U) {
        m_buffer[2U] = CMD_DEBUG_DUMP;
        sendNAK(RSN_ILLEGAL_LENGTH);
        return;
    }

    uint8_t reply[516U];
    ::memset(reply, 0x00U, 516U);

    if (length > 252U) {
        reply[0U] = DVM_LONG_FRAME_START;
        reply[1U] = (length >> 8U) & 0xFFU;
        reply[2U] = (length & 0xFFU);
        reply[3U] = CMD_DEBUG_DUMP;

        for (uint8_t i = 0U; i < length; i++)
            reply[i + 4U] = data[i];

        writeInt(1U, reply, length + 4U);
    }
    else {
        reply[0U] = DVM_SHORT_FRAME_START;
        reply[1U] = length + 3U;
        reply[2U] = CMD_DEBUG_DUMP;

        for (uint8_t i = 0U; i < length; i++)
            reply[i + 3U] = data[i];

        writeInt(1U, reply, length + 3U);
    }
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Write acknowlegement. */

void SerialPort::sendACK()
{
    uint8_t reply[4U];

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 4U;
    reply[2U] = CMD_ACK;
    reply[3U] = m_buffer[2U];

    writeInt(1U, reply, 4);
}

/* Write negative acknowlegement. */

void SerialPort::sendNAK(uint8_t err)
{
    uint8_t reply[5U];

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 5U;
    reply[2U] = CMD_NAK;
    reply[3U] = m_buffer[2U];
    reply[4U] = err;

    writeInt(1U, reply, 5);
}

/* Write modem DSP status. */

void SerialPort::getStatus()
{
    m_modem->m_io.resetWatchdog();

    uint8_t reply[15U];

    // send all sorts of interesting internal values
    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 12U;
    reply[2U] = CMD_GET_STATUS;

    reply[3U] = 0x00U;
    if (m_modem->m_dmrEnable)
        reply[3U] |= 0x02U;
    if (m_modem->m_p25Enable)
        reply[3U] |= 0x08U;
    if (m_modem->m_nxdnEnable)
        reply[3U] |= 0x10U;

    reply[4U] = uint8_t(m_modem->m_modemState);

    reply[5U] = m_modem->m_tx ? 0x01U : 0x00U;

    bool adcOverflow;
    bool dacOverflow;
    m_modem->m_io.getOverflow(adcOverflow, dacOverflow);

    if (adcOverflow)
        reply[5U] |= 0x02U;

    if (m_modem->m_io.hasRXOverflow())
        reply[5U] |= 0x04U;

    if (m_modem->m_io.hasTXOverflow())
        reply[5U] |= 0x08U;

    if (m_modem->m_io.hasLockout())
        reply[5U] |= 0x10U;

    if (dacOverflow)
        reply[5U] |= 0x20U;

    reply[5U] |= m_modem->m_dcd ? 0x40U : 0x00U;

    reply[6U] = 0U;

    if (m_modem->m_dmrEnable) {
        if (m_modem->m_duplex) {
            reply[7U] = m_modem->m_dmrTX.getSpace1();
            reply[8U] = m_modem->m_dmrTX.getSpace2();
        }
        else {
            reply[7U] = 10U;
            reply[8U] = m_modem->m_dmrDMOTX.getSpace();
        }
    }
    else {
        reply[7U] = 0U;
        reply[8U] = 0U;
    }

    reply[9U] = 0U;

    if (m_modem->m_p25Enable)
        reply[10U] = m_modem->m_p25TX.getSpace();
    else
        reply[10U] = 0U;

    if (m_modem->m_nxdnEnable)
        reply[11U] = m_modem->m_nxdnTX.getSpace();
    else
        reply[11U] = 0U;

    writeInt(1U, reply, 12);
}

/* Write modem DSP version. */

void SerialPort::getVersion()
{
    uint8_t reply[200U];

    reply[0U] = DVM_SHORT_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = CMD_GET_VERSION;

    reply[3U] = PROTOCOL_VERSION;

    reply[4U] = m_modem->m_io.getCPU();

    // Reserve 16 bytes for the UDID
    ::memset(reply + 5U, 0x00U, 16U);
    m_modem->m_io.getUDID(reply + 5U);

    uint8_t count = 21U;
    for (uint8_t i = 0U; HARDWARE[i] != 0x00U; i++, count++)
        reply[count] = HARDWARE[i];

    reply[1U] = count;

    writeInt(1U, reply, count);
}

/* Helper to validate the passed modem state is valid. */

uint8_t SerialPort::modemStateCheck(DVM_STATE state)
{
    // invalid mode check
    if (state != STATE_IDLE && state != STATE_DMR && state != STATE_P25 && state != STATE_NXDN && 
        state != STATE_P25_CAL_1K &&
        state != STATE_DMR_DMO_CAL_1K && state != STATE_DMR_CAL_1K &&
        state != STATE_DMR_LF_CAL &&
        state != STATE_RSSI_CAL &&
        state != STATE_P25_CAL && state != STATE_DMR_CAL &&
        state != STATE_NXDN_CAL)
        return RSN_INVALID_MODE;
/*
    // DMR without DMR being enabled
    if (state == STATE_DMR && !m_dmrEnable)
        return RSN_DMR_DISABLED;
    // P25 without P25 being enabled
    if (state == STATE_P25 && !m_p25Enable)
        return RSN_P25_DISABLED;
    // NXDN without NXDN being enabled
    if (state == STATE_NXDN && !m_nxdnEnable)
        return RSN_NXDN_DISABLED;
*/
    return RSN_OK;
}

/* Set modem DSP configuration from serial port data. */

uint8_t SerialPort::setConfig(const uint8_t* data, uint8_t length)
{
    if (length < 21U)
        return RSN_ILLEGAL_LENGTH;

    bool rxInvert = (data[0U] & 0x01U) == 0x01U;
    bool txInvert = (data[0U] & 0x02U) == 0x02U;
    bool pttInvert = (data[0U] & 0x04U) == 0x04U;
    bool simplex = (data[0U] & 0x80U) == 0x80U;

    m_debug = (data[0U] & 0x10U) == 0x10U;
    m_modem->m_debug = m_debug; // should we be doing this?

    bool dcBlockerEnable = (data[1U] & 0x01U) == 0x01U;
    bool cosLockoutEnable = (data[1U] & 0x04U) == 0x04U;

    bool dmrEnable = (data[1U] & 0x02U) == 0x02U;
    bool p25Enable = (data[1U] & 0x08U) == 0x08U;
    bool nxdnEnable = (data[1U] & 0x10U) == 0x10U;

    uint8_t fdmaPreamble = data[2U];
    if (fdmaPreamble > 255U)
        return RSN_INVALID_FDMA_PREAMBLE;

    DVM_STATE modemState = DVM_STATE(data[3U]);

    uint8_t ret = modemStateCheck(modemState);
    if (ret != RSN_OK)
        return ret;

    uint8_t rxLevel = data[4U];

    uint8_t colorCode = data[6U];
    if (colorCode > 15U)
        return RSN_INVALID_DMR_CC;

    uint8_t dmrRxDelay = data[7U];
    if (dmrRxDelay > 255U)
        return RSN_INVALID_DMR_RX_DELAY;

    uint16_t nac = (data[8U] << 4) + (data[9U] >> 4);

    uint8_t cwIdTXLevel = data[5U];
    uint8_t dmrTXLevel = data[10U];
    uint8_t p25TXLevel = data[12U];
    uint8_t nxdnTXLevel = data[15U];

    uint8_t p25CorrCount = data[11U];
    if (p25CorrCount > 255U)
        return RSN_INVALID_P25_CORR_COUNT;

    //uint8_t nxdnCorrCount = data[22U];

    m_modem->m_modemState = modemState;

    m_modem->m_dcBlockerEnable = dcBlockerEnable;
    m_modem->m_cosLockoutEnable = cosLockoutEnable;

    m_modem->m_dmrEnable = dmrEnable;
    m_modem->m_p25Enable = p25Enable;
    m_modem->m_nxdnEnable = nxdnEnable;
    m_modem->m_duplex = !simplex;

    m_modem->m_p25TX.setPreambleCount(fdmaPreamble);
    m_modem->m_dmrDMOTX.setPreambleCount(fdmaPreamble);
    //m_modem->m_nxdnTX.setPreambleCount(fdmaPreamble);

    m_modem->m_p25RX.setNAC(nac);
    m_modem->m_p25RX.setCorrCount(p25CorrCount);

    m_modem->m_dmrTX.setColorCode(colorCode);
    m_modem->m_dmrRX.setColorCode(colorCode);
    m_modem->m_dmrRX.setRxDelay(dmrRxDelay);
    m_modem->m_dmrDMORX.setColorCode(colorCode);
    m_modem->m_dmrIdleRX.setColorCode(colorCode);

    //m_modem->m_nxdnRX.setCorrCount(nxdnCorrCount);

    m_modem->m_io.setParameters(rxInvert, txInvert, pttInvert, rxLevel, cwIdTXLevel, dmrTXLevel, p25TXLevel, nxdnTXLevel);

    setMode(m_modem->m_modemState);

    m_modem->m_io.start();

    return RSN_OK;
}

/* Set modem DSP mode from serial port data. */

uint8_t SerialPort::setMode(const uint8_t* data, uint8_t length)
{
    if (length < 1U)
        return RSN_ILLEGAL_LENGTH;

    DVM_STATE modemState = DVM_STATE(data[0U]);

    if (modemState == m_modem->m_modemState)
        return RSN_OK;

    uint8_t ret = modemStateCheck(modemState);
    if (ret != RSN_OK)
        return ret;

    setMode(modemState);

    return RSN_OK;
}

/* Sets the modem state. */

void SerialPort::setMode(DVM_STATE modemState)
{
    switch (modemState) {
    case STATE_DMR:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to DMR");
        m_modem->writeDebug("SerialPort::setMode() mode set to DMR");
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_P25:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to P25");
        m_modem->writeDebug("SerialPort::setMode() mode set to P25");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_NXDN:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to NXDN");
        m_modem->writeDebug("SerialPort::setMode() mode set to NXDN");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_DMR_CAL:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to DMR Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to DMR Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_P25_CAL:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to P25 Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to P25 Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_NXDN_CAL:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to NXDN Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to NXDN Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_RSSI_CAL:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to RSSI Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to RSSI Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_DMR_LF_CAL:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to DMR 80Hz Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to DMR 80Hz Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_DMR_CAL_1K:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to DMR BS 1031Hz Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to DMR BS 1031Hz Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_DMR_DMO_CAL_1K:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to DMR DMO 1031Hz Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to DMR MS 1031Hz Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    case STATE_P25_CAL_1K:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to P25 1011Hz Calibrate");
        m_modem->writeDebug("SerialPort::setMode() mode set to P25 1011Hz Calibrate");
        m_modem->m_dmrIdleRX.reset();
        m_modem->m_dmrDMORX.reset();
        m_modem->m_dmrRX.reset();
        m_modem->m_p25RX.reset();
        m_modem->m_nxdnRX.reset();
        m_modem->m_cwIdTX.reset();
        break;
    default:
        if (m_modem->m_debug)
            ::LogDebugEx(LOG_SERIAL, "SerialPort::setMode()", "mode set to Idle");
        m_modem->writeDebug("SerialPort::setMode() mode set to Idle");
        // STATE_IDLE
        break;
    }

    m_modem->m_modemState = modemState;

    m_modem->m_io.setMode();
}

/* Sets the fine-tune symbol levels. */

uint8_t SerialPort::setSymbolLvlAdj(const uint8_t* data, uint8_t length)
{
    if (length < 6U)
        return RSN_ILLEGAL_LENGTH;

    int8_t dmrSymLvl3Adj = int8_t(data[0U]) - 128;
    if (dmrSymLvl3Adj > 128)
        return RSN_INVALID_REQUEST;
    if (dmrSymLvl3Adj < -128)
        return RSN_INVALID_REQUEST;

    int8_t dmrSymLvl1Adj = int8_t(data[1U]) - 128;
    if (dmrSymLvl1Adj > 128)
        return RSN_INVALID_REQUEST;
    if (dmrSymLvl1Adj < -128)
        return RSN_INVALID_REQUEST;

    int8_t p25SymLvl3Adj = int8_t(data[2U]) - 128;
    if (p25SymLvl3Adj > 128)
        return RSN_INVALID_REQUEST;
    if (p25SymLvl3Adj < -128)
        return RSN_INVALID_REQUEST;

    int8_t p25SymLvl1Adj = int8_t(data[3U]) - 128;
    if (p25SymLvl1Adj > 128)
        return RSN_INVALID_REQUEST;
    if (p25SymLvl1Adj < -128)
        return RSN_INVALID_REQUEST;

    int8_t nxdnSymLvl3Adj = int8_t(data[4U]) - 128;
    if (nxdnSymLvl3Adj > 128)
        return RSN_INVALID_REQUEST;
    if (nxdnSymLvl3Adj < -128)
        return RSN_INVALID_REQUEST;

    int8_t nxdnSymLvl1Adj = int8_t(data[5U]) - 128;
    if (nxdnSymLvl1Adj > 128)
        return RSN_INVALID_REQUEST;
    if (nxdnSymLvl1Adj < -128)
        return RSN_INVALID_REQUEST;

    m_modem->m_p25TX.setSymbolLvlAdj(p25SymLvl3Adj, p25SymLvl1Adj);

    m_modem->m_dmrDMOTX.setSymbolLvlAdj(dmrSymLvl3Adj, dmrSymLvl1Adj);
    m_modem->m_dmrTX.setSymbolLvlAdj(dmrSymLvl3Adj, dmrSymLvl1Adj);

    m_modem->m_nxdnTX.setSymbolLvlAdj(nxdnSymLvl3Adj, nxdnSymLvl1Adj);

    return RSN_OK;
}

/* Sets the software Rx sample level. */

uint8_t SerialPort::setRXLevel(const uint8_t* data, uint8_t length)
{
    if (length < 1U)
        return RSN_ILLEGAL_LENGTH;

    uint8_t rxLevel = data[0U];

    m_modem->m_io.setRXLevel(rxLevel);

    return RSN_OK;
}

/* Sets the RF parameters. */

uint8_t SerialPort::setRFParams(const uint8_t* data, uint8_t length)
{
    if (length < 17U)
        return RSN_ILLEGAL_LENGTH;

    uint32_t rxFreq, txFreq;
    uint8_t rfPower;

    rxFreq = data[1U] << 0;
    rxFreq |= data[2U] << 8;
    rxFreq |= data[3U] << 16;
    rxFreq |= data[4U] << 24;

    txFreq = data[5U] << 0;
    txFreq |= data[6U] << 8;
    txFreq |= data[7U] << 16;
    txFreq |= data[8U] << 24;

    rfPower = data[9U];

    int8_t dmrDiscBWAdj = int8_t(data[10U]) - 128;
    if (dmrDiscBWAdj > 128)
        return RSN_INVALID_REQUEST;
    if (dmrDiscBWAdj < -128)
        return RSN_INVALID_REQUEST;

    int8_t p25DiscBWAdj = int8_t(data[11U]) - 128;
    if (p25DiscBWAdj > 128)
        return RSN_INVALID_REQUEST;
    if (p25DiscBWAdj < -128)
        return RSN_INVALID_REQUEST;

    int8_t nxdnDiscBWAdj = int8_t(data[15U]) - 128;
    if (nxdnDiscBWAdj > 128)
        return RSN_INVALID_REQUEST;
    if (nxdnDiscBWAdj < -128)
        return RSN_INVALID_REQUEST;

    int8_t dmrPostBWAdj = int8_t(data[12U]) - 128;
    if (dmrPostBWAdj > 128)
        return RSN_INVALID_REQUEST;
    if (dmrPostBWAdj < -128)
        return RSN_INVALID_REQUEST;

    int8_t p25PostBWAdj = int8_t(data[13U]) - 128;
    if (p25PostBWAdj > 128)
        return RSN_INVALID_REQUEST;
    if (p25PostBWAdj < -128)
        return RSN_INVALID_REQUEST;

    int8_t nxdnPostBWAdj = int8_t(data[16U]) - 128;
    if (nxdnPostBWAdj > 128)
        return RSN_INVALID_REQUEST;
    if (nxdnPostBWAdj < -128)
        return RSN_INVALID_REQUEST;

    // support optional AFC parameters
    if (length > 17U) {
        if (length < 19U)
            return RSN_ILLEGAL_LENGTH;

        bool afcEnable = (data[17U] & 0x80U) == 0x80U;
        uint8_t afcKI = data[17U] & 0x0FU;
        uint8_t afcKP = (data[17U] >> 4) & 0x07U;
        uint8_t afcRange = data[18U];

        m_modem->m_io.setAFCParams(afcEnable, afcKI, afcKP, afcRange);
    } else {
        m_modem->m_io.setAFCParams(false, 11, 4, 1);
    }

    m_modem->m_io.setRFAdjust(dmrDiscBWAdj, p25DiscBWAdj, nxdnDiscBWAdj, dmrPostBWAdj, p25PostBWAdj, nxdnPostBWAdj);

    return m_modem->m_io.setRFParams(rxFreq, txFreq, rfPower);
}

/* Sets the protocol ring buffer sizes. */

uint8_t SerialPort::setBuffers(const uint8_t* data, uint8_t length)
{
    if (length < 1U)
        return RSN_ILLEGAL_LENGTH;
    if (m_modem->m_modemState != STATE_IDLE)
        return RSN_INVALID_MODE;

    uint16_t dmrBufSize = dmr::DMR_TX_BUFFER_LEN;
    uint16_t p25BufSize = p25::P25_TX_BUFFER_LEN;
    uint16_t nxdnBufSize = nxdn::NXDN_TX_BUFFER_LEN;

    dmrBufSize = (data[0U] << 8) + (data[1U]);
    p25BufSize = (data[2U] << 8) + (data[3U]);
    nxdnBufSize = (data[4U] << 8) + (data[5U]);

    m_modem->m_p25TX.resizeBuffer(p25BufSize);
    m_modem->m_nxdnTX.resizeBuffer(nxdnBufSize);

    m_modem->m_dmrTX.resizeBuffer(dmrBufSize);
    m_modem->m_dmrDMOTX.resizeBuffer(dmrBufSize);

    return RSN_OK;
}

/* Reads data from the modem flash parititon. */

void SerialPort::flashRead()
{
    m_modem->writeDebug("SerialPort: flashRead(): unsupported on Native SDR");
    sendNAK(RSN_NO_INTERNAL_FLASH);
    // unused on SDR based dedicated modems
}

/* Writes data to the modem flash partition. */

uint8_t SerialPort::flashWrite(const uint8_t* data, uint8_t length)
{
    m_modem->writeDebug("SerialPort: flashWrite(): unsupported on Native SDR");
    // unused on SDR based dedicated modems
    return RSN_NO_INTERNAL_FLASH;
}

/* */

void SerialPort::beginInt(uint8_t n, int speed)
{
    switch (n) {
    case 1U:
        s_readBuffer = 0x00U;
        break;
    default:
        break;
    }
}

/* */

int SerialPort::availableInt(uint8_t n)
{
    switch (n) {
    case 1U:
        return m_modem->m_port->read(&s_readBuffer, (uint8_t)(1 * sizeof(uint8_t)));
    default:
        return 0;
    }
}

/* */

int SerialPort::availableForWriteInt(uint8_t n)
{
    switch (n) {
    case 1U:
        return true;
    default:
        return false;
    }
}

/* */

uint8_t SerialPort::readInt(uint8_t n)
{
    switch (n) {
    case 1U:
        return s_readBuffer;
    default:
        return 0U;
    }
}

/* */

void SerialPort::writeInt(uint8_t n, const uint8_t* data, uint16_t length, bool flush)
{
    switch (n) {
    case 1U:
        m_modem->m_port->write(data, length);
        break;
    default:
        break;
    }
}
