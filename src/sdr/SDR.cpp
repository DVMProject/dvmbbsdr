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
#include "modem/Modem.h"
#include "modem/port/UARTPort.h"
#include "modem/port/PseudoPTYPort.h"
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

#include <sys/utsname.h>
#include <unistd.h>
#include <pwd.h>

using namespace modem;

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
    m_modems(),
    m_trace(false),
    m_debug(false)
{
    /* stub */
}

/* Finalizes a instance of the SDR class. */

SDR::~SDR() = default;

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
        ">> SDR Daemon\r\n");

    // read base parameters from configuration
    ret = readParams();
    if (!ret)
        return EXIT_FAILURE;

    // initialize modems
    ret = createModems();
    if (!ret)
        return EXIT_FAILURE;

    yaml::Node systemConf = m_conf["system"];

    ::LogInfoEx(LOG_HOST, "SDR is up and running");

    s_running = true;

    StopWatch stopWatch;
    stopWatch.start();

    // main execution loop
    struct utsname utsinfo;
    ::memset(&utsinfo, 0, sizeof(utsinfo));
    ::uname(&utsinfo);

    ::LogInfoEx(LOG_HOST, "[ OK ] SDR is up and running on %s %s %s", utsinfo.sysname, utsinfo.release, utsinfo.machine);

    while (!g_killed) {
        uint32_t ms = stopWatch.elapsed();

        ms = stopWatch.elapsed();
        stopWatch.start();

        if (ms < 2U)
            Thread::sleep(1U);
    }

    s_running = false;

    for (auto modem : m_modems) {
        if (modem != nullptr) {
            modem->close();
        }
    }

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

/* Initializes the virtual modems. */

bool SDR::createModems()
{
    yaml::Node& modemList = m_conf["modems"];
    if (modemList.size() > 0U) {
        for (size_t i = 0; i < modemList.size(); i++) {
            yaml::Node& modemConf = modemList[i];

            yaml::Node modemProtocol = modemConf["protocol"];
            std::string uartPort = modemProtocol["port"].as<std::string>();
            uint32_t uartSpeed = modemProtocol["speed"].as<uint32_t>(115200);

            bool trace = modemConf["trace"].as<bool>(false);
            bool debug = modemConf["debug"].as<bool>(false);

            // if modem debug is being forced from the commandline -- enable modem debug
            if (g_debug) {
                debug = true;
            }

            LogInfo("Modem %u Parameters", i + 1);
            LogInfo("    Port Type: %s", PTY_PORT);
            LogInfo("    PTY Port: %s", uartPort.c_str());
            LogInfo("    PTY Speed: %u", uartSpeed);

            if (debug) {
                LogInfo("    Debug: yes");
            }

            port::IModemPort* modemPort = nullptr;
            port::SERIAL_SPEED serialSpeed = port::SERIAL_115200;
            switch (uartSpeed) {
            case 1200:
                serialSpeed = port::SERIAL_1200;
                break;
            case 2400:
                serialSpeed = port::SERIAL_2400;
                break;
            case 4800:
                serialSpeed = port::SERIAL_4800;
                break;
            case 9600:
                serialSpeed = port::SERIAL_9600;
                break;
            case 19200:
                serialSpeed = port::SERIAL_19200;
                break;
            case 38400:
                serialSpeed = port::SERIAL_38400;
                break;
            case 76800:
                serialSpeed = port::SERIAL_76800;
                break;
            case 230400:
                serialSpeed = port::SERIAL_230400;
                break;
            case 460800:
                serialSpeed = port::SERIAL_460800;
                break;
            default:
                LogWarning(LOG_HOST, "Unsupported serial speed %u, defaulting to %u", uartSpeed, port::SERIAL_115200);
                uartSpeed = 115200;
            case 115200:
                break;
            }

            modemPort = new port::PseudoPTYPort(uartPort, serialSpeed, false);

            Modem* modem = new Modem(modemPort, trace, debug);

            bool ret = modem->open();
            if (!ret) {
                LogError(LOG_HOST, "Failed to open PTY for %s", uartPort.c_str());
                delete modem;
                modem = nullptr;
            }

            m_modems.push_back(modem);
        }
    }

    return true;
}
