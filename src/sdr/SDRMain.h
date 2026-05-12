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
 * @file SDRMain.h
 * @ingroup modem_fw
 * @file SDRMain.cpp
 * @ingroup modem_fw
 */
#if !defined(__SDR_MAIN_H__)
#define __SDR_MAIN_H__

#include "Defines.h"

#include <string>

// ---------------------------------------------------------------------------
//  Externs
// ---------------------------------------------------------------------------

/** @brief  */
extern int g_signal;
/** @brief  */
extern std::string g_progExe;
/** @brief  */
extern std::string g_iniFile;

/** @brief (Global) Flag indicating foreground operation. */
extern bool g_foreground;
/** @brief (Global) Flag indicating the FNE should stop immediately. */
extern bool g_killed;
/** @brief (Global) Flag indicating debug mode. */
extern bool g_debug;

extern uint8_t* g_gitHashBytes;

/**
 * @brief Helper to trigger a fatal error message. This will cause the program to terminate 
 * immediately with an error message.
 * 
 * @param msg String format.
 * 
 * This is a variable argument function.
 */
extern HOST_SW_API void fatal(const char* msg, ...);

#endif // __SDR_MAIN_H__
