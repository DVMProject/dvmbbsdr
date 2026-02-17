// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Modem Firmware
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2025-2026 Bryan Biedenkapp, N2PLL
 *
 */
#include "Defines.h"
#include "common/StopWatch.h"
#include "common/Thread.h"
#include "common/Log.h"
#include "common/Utils.h"
#include "SDR.h"
#include "SDRMain.h"

#include <cstdio>
#include <algorithm>
#include <functional>
#include <random>

#include <unistd.h>
#include <pwd.h>

// ---------------------------------------------------------------------------
//  Static Class Members
// ---------------------------------------------------------------------------

bool SDR::s_running = false;

// ---------------------------------------------------------------------------
//  Public Class Members
// ---------------------------------------------------------------------------

/* Initializes a new instance of the SDR class. */

SDR::SDR(const std::string& confFile) :
    m_confFile(confFile),
    m_conf(),
    m_trace(false),
    m_debug(false)
{
    /* stub */
}

/* Finalizes a instance of the SDR class. */

SDR::~SDR()
{
    /* stub */
}

/* Executes the main SDR processing loop. */

int SDR::run()
{
    bool ret = false;
    try {
        ret = yaml::Parse(m_conf, m_confFile.c_str());
        if (!ret) {
            ::fatal("cannot read the configuration file, %s\n", m_confFile.c_str());
        }
    }
    catch (yaml::OperationException const& e) {
        ::fatal("cannot read the configuration file - %s (%s)", m_confFile.c_str(), e.message());
    }

    bool m_daemon = m_conf["daemon"].as<bool>(false);
    if (m_daemon && g_foreground)
        m_daemon = false;

    // initialize system logging
    yaml::Node logConf = m_conf["log"];
    ret = ::LogInitialise(logConf["filePath"].as<std::string>(), logConf["fileRoot"].as<std::string>(),
        logConf["fileLevel"].as<uint32_t>(0U), logConf["displayLevel"].as<uint32_t>(0U));
    if (!ret) {
        ::fatal("unable to open the log file\n");
    }

#if !defined(_WIN32)
    // handle POSIX process forking
    if (m_daemon) {
        // create new process
        pid_t pid = ::fork();
        if (pid == -1) {
            ::fprintf(stderr, "%s: Couldn't fork() , exiting\n", g_progExe.c_str());
            ::LogFinalise();
            return EXIT_FAILURE;
        }
        else if (pid != 0) {
            ::LogFinalise();
            exit(EXIT_SUCCESS);
        }

        // create new session and process group
        if (::setsid() == -1) {
            ::fprintf(stderr, "%s: Couldn't setsid(), exiting\n", g_progExe.c_str());
            ::LogFinalise();
            return EXIT_FAILURE;
        }

        // set the working directory to the root directory
        if (::chdir("/") == -1) {
            ::fprintf(stderr, "%s: Couldn't cd /, exiting\n", g_progExe.c_str());
            ::LogFinalise();
            return EXIT_FAILURE;
        }

        ::close(STDIN_FILENO);
        ::close(STDOUT_FILENO);
        ::close(STDERR_FILENO);
    }
#endif // !defined(_WIN32)

    ::LogInfo(__BANNER__ "\r\n" "" DESCRIPTION " " __VER__ " (built " __BUILD__ ")\r\n" \
        "Copyright (c) 2025-2026 Bryan Biedenkapp, N2PLL and DVMProject (https://github.com/dvmproject) Authors.\r\n" \
        "Portions Copyright (c) 2015-2021 by Jonathan Naylor, G4KLX and others\r\n" \
        ">> Talkgroup Patch\r\n");

    // read base parameters from configuration
    ret = readParams();
    if (!ret)
        return EXIT_FAILURE;

    yaml::Node systemConf = m_conf["system"];

    ::LogInfoEx(LOG_HOST, "SDR is up and running");

    s_running = true;

    StopWatch stopWatch;
    stopWatch.start();

    // main execution loop
    while (!g_killed) {
        uint32_t ms = stopWatch.elapsed();

        ms = stopWatch.elapsed();
        stopWatch.start();

        if (ms < 2U)
            Thread::sleep(1U);
    }

    s_running = false;

    return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
//  Private Class Members
// ---------------------------------------------------------------------------

/* Reads basic configuration parameters from the YAML configuration file. */

bool SDR::readParams()
{
    yaml::Node systemConf = m_conf["system"];

    m_trace = systemConf["trace"].as<bool>(false);
    m_debug = systemConf["debug"].as<bool>(false);

    LogInfo("General Parameters");

    if (m_debug) {
        LogInfo("    Debug: yes");
    }

    return true;
}
