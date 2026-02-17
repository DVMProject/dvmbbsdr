// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Modem Firmware
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2015,2016,2017 Jonathan Naylor, G4KLX
 *  Copyright (C) 2018,2025 Bryan Biedenkapp, N2PLL
 *
 */
/**
 * @file Defines.h
 * @ingroup modem_fw
 */
#if !defined(__DEFINES_H__)
#define __DEFINES_H__

#include <stdint.h>

#include "common/Defines.h"
#include "common/GitHash.h"

#include <cstring>

#include "modem/arm_math.h"

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

#undef __PROG_NAME__
#define __PROG_NAME__ "Digital Voice Modem SDR"
#define __NET_NAME__ "DVM_DMR_P25"
#undef __EXE_NAME__
#define __EXE_NAME__ "dvmbbsdr"

#define DSP_FW_API 

// Normal Boxcar Filter for P25
//#define P25_RX_NORMAL_BOXCAR

// Narrow Boxcar Filter for P25
#define P25_RX_NARROW_BOXCAR

// Boxcar Filter for NXDN
//#define NXDN_BOXCAR_FILTER

// Alternate P25 Symbol Levels
//#define P25_ALTERNATE_SYM_LEVELS

// Allow for the use of high quality external clock oscillators
// The number is the frequency of the oscillator in Hertz.
//
// The frequency of the TCXO must be an integer multiple of 48000.
// Frequencies such as 12.0 Mhz (48000 * 250) and 14.4 Mhz (48000 * 300) are suitable.
// Frequencies such as 10.0 Mhz (48000 * 208.333) or 20 Mhz (48000 * 416.666) are not suitable.
//
#ifndef EXTERNAL_OSC
#define EXTERNAL_OSC 12000000
#endif

// Sanity check to make sure EXTERNAL_OSC is a valid integer multiple of 48kHz
#if (EXTERNAL_OSC % 48000 != 0)
#error "Invalid EXTERNAL_OSC specified! Must be an integer multiple of 48000"
#endif

// Pass RSSI information to the host
// #define SEND_RSSI_DATA

#define DESCR_DMR        "DMR, "
#define DESCR_P25        "P25, "
#define DESCR_NXDN       "NXDN, "

#if defined(EXTERNAL_OSC)
#define DESCR_OSC        "TCXO, "
#else
#define DESCR_OSC        ""
#endif

#if defined(SEND_RSSI_DATA)
#define DESCR_RSSI        "RSSI, "
#else
#define DESCR_RSSI        ""
#endif

#define DESCRIPTION        __PROG_NAME__ " (" DESCR_DMR DESCR_P25 DESCR_NXDN DESCR_OSC DESCR_RSSI "CW Id)"

#define CPU_TYPE_NATIVE_SDR 0xF0U

// ---------------------------------------------------------------------------
//  Class Prototypes
// ---------------------------------------------------------------------------

namespace modem { class DSP_FW_API Modem; }

#endif // __DEFINES_H__
