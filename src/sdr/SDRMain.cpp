// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Modem Firmware
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2015,2016,2017,2018 Jonathan Naylor, G4KLX
 *  Copyright (C) 2016 Mathis Schmieder, DB9MAT
 *  Copyright (C) 2016 Colin Durbridge, G4EML
 *  Copyright (C) 2018,2024,2025 Bryan Biedenkapp, N2PLL
 *
 */
#include "Globals.h"
#include "common/Log.h"
#include "common/Utils.h"
#include "modem/port/PseudoPTYPort.h"

#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pwd.h>
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
//	Macros
// ---------------------------------------------------------------------------

#define IS(s) (::strcmp(argv[i], s) == 0)

// ---------------------------------------------------------------------------
//  Globals Variables
// ---------------------------------------------------------------------------

DVM_STATE m_modemState = STATE_IDLE;

bool m_dmrEnable = true;
bool m_p25Enable = true;
bool m_nxdnEnable = true;

bool m_dcBlockerEnable = true;
bool m_cosLockoutEnable = false;

bool m_duplex = true;

bool m_tx = false;
bool m_dcd = false;

/* DMR BS */
dmr::DMRIdleRX dmrIdleRX;
dmr::DMRRX dmrRX;
dmr::DMRTX dmrTX;

/* DMR MS-DMO */
dmr::DMRDMORX dmrDMORX;
dmr::DMRDMOTX dmrDMOTX;

/* P25 */
p25::P25RX p25RX;
p25::P25TX p25TX;

/* NXDN */
nxdn::NXDNRX nxdnRX;
nxdn::NXDNTX nxdnTX;

/* Calibration */
dmr::CalDMR calDMR;
p25::CalP25 calP25;
nxdn::CalNXDN calNXDN;
CalRSSI calRSSI;

/* CW */
CWIdTX cwIdTX;

/* RS232 and Air Interface I/O */
modem::SerialPortSDR serial;
IO io;

std::string g_progExe = std::string(__EXE_NAME__);

std::string m_ptyPort = std::string("/dev/ptmx");

std::string g_logFileName = std::string("dsp.log");

bool g_debug = false;

int g_signal = 0;
bool g_killed = false;

bool g_daemon = false;

extern modem::port::PseudoPTYPort* m_serialPort;

// ---------------------------------------------------------------------------
//  Global Functions
// ---------------------------------------------------------------------------

void setup()
{
    io.init();

    serial.start();
}

void loop()
{
    serial.process();

    io.process();

    // The following is for transmitting
    if (m_dmrEnable && m_modemState == STATE_DMR) {
        if (m_duplex)
            dmrTX.process();
        else
            dmrDMOTX.process();
    }

    if (m_p25Enable && m_modemState == STATE_P25)
        p25TX.process();

    if (m_nxdnEnable && m_modemState == STATE_NXDN)
        nxdnTX.process();

    if (m_modemState == STATE_DMR_DMO_CAL_1K || m_modemState == STATE_DMR_CAL_1K ||
        m_modemState == STATE_DMR_LF_CAL || m_modemState == STATE_DMR_CAL)
        calDMR.process();

    if (m_modemState == STATE_P25_CAL_1K || m_modemState == STATE_P25_CAL)
        calP25.process();

    if (m_modemState == STATE_NXDN_CAL)
        calNXDN.process();

    if (m_modemState == STATE_CW || m_modemState == STATE_IDLE)
        cwIdTX.process();
}

void fatal(const char* message)
{
    ::fprintf(stderr, "%s: %s\n", g_progExe.c_str(), message);
    exit(EXIT_FAILURE);
}

void usage(const char* message, const char* arg)
{
    ::fprintf(stdout, "" DESCRIPTION " (built %s)\r\n", __BUILD__);
    ::fprintf(stdout, "Copyright (c) 2025 Bryan Biedenkapp, N2PLL and DVMProject (https://github.com/dvmproject) Authors.\n");
    ::fprintf(stdout, "Portions Copyright (c) 2015-2021 by Jonathan Naylor, G4KLX and others\n\n");
    if (message != nullptr) {
        ::fprintf(stderr, "%s: ", g_progExe.c_str());
        ::fprintf(stderr, message, arg);
        ::fprintf(stderr, "\n\n");
    }

    ::fprintf(stdout, 
        "usage: %s [-bdvh]"
        " [--syslog]" 
        " [-p <PTY port>]"
        " [-l <log filename>]\n\n"
        "  -b       background process\n"
        "\n"
        "  -d       enable debug\n"
        "  -v       show version information\n"
        "  -h       show this screen\n"
        "\n"
        "  --syslog  force logging to syslog\n"
        "\n"
        "  -p       PTY Port\n"
        "\n"
        "  -l       Log Filename\n"
        "\n"
        "  --       stop handling options\n",
        g_progExe.c_str());
    exit(EXIT_FAILURE);
}

int checkArgs(int argc, char* argv[])
{
    int i, p = 0;

    // iterate through arguments
    for (i = 1; i <= argc; i++)
    {
        if (argv[i] == nullptr) {
            break;
        }

        if (*argv[i] != '-') {
            continue;
        }
        else if (IS("--")) {
            ++p;
            break;
        }
        else if (IS("-p")) {
            if ((argc - 1) <= 0)
                usage("error: %s", "must specify the PTY port");
            m_ptyPort = std::string(argv[++i]);

            if (m_ptyPort == "")
                usage("error: %s", "PTY port cannot be blank!");

            p += 2;
        }
        else if (IS("-l")) {
            if ((argc - 1) <= 0)
                usage("error: %s", "must specify the log filename");
            g_logFileName = std::string(argv[++i]);

            if (g_logFileName == "")
                usage("error: %s", "log filename cannot be blank!");

            p += 2;
        }
        else if (IS("-b")) {
            ++p;
            g_daemon = true;
        }
        else if (IS("-d")) {
            ++p;
            g_debug = true;
        }
        else if (IS("--syslog")) {
            g_useSyslog = true;
        }
        else if (IS("-v")) {
            ::fprintf(stdout, "" DESCRIPTION " (built %s)\r\n", __BUILD__);
            ::fprintf(stdout, "Copyright (c) 2022 Bryan Biedenkapp, N2PLL and DVMProject (https://github.com/dvmproject) Authors.\r\n");
            ::fprintf(stdout, "Portions Copyright (c) 2015-2021 by Jonathan Naylor, G4KLX and others\r\n");
            if (argc == 2)
                exit(EXIT_SUCCESS);
        }
        else if (IS("-h")) {
            usage(nullptr, nullptr);
            if (argc == 2)
                exit(EXIT_SUCCESS);
        }
        else {
            usage("unrecognized option `%s'", argv[i]);
        }
    }

    if (p < 0 || p > argc) {
        p = 0;
    }

    return ++p;
}

static void sigHandler(int signum)
{
    g_killed = true;
    g_signal = signum;
}

// ---------------------------------------------------------------------------
//  Program Entry Point
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    if (argv[0] != nullptr && *argv[0] != 0)
        g_progExe = std::string(argv[0]);

    if (argc > 1) {
        // check arguments
        int i = checkArgs(argc, argv);
        if (i < argc) {
            argc -= i;
            argv += i;
        }
        else {
            argc--;
            argv++;
        }
    }

    ::signal(SIGINT, sigHandler);
    ::signal(SIGTERM, sigHandler);
    ::signal(SIGHUP, sigHandler);

    // initialize system logging
    bool ret = ::LogInitialise(".", g_logFileName.c_str(), 1U, 1U);
    if (!ret) {
        ::fprintf(stderr, "unable to open the log file\n");
        return 1;
    }

    // handle POSIX process forking
    if (g_daemon) {
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

    do {
        g_signal = 0;

        {
            ::LogInfo("" DESCRIPTION " (built %s)", __BUILD__);
            ::LogInfo("Copyright (c) 2017-2025 Bryan Biedenkapp, N2PLL and DVMProject (https://github.com/dvmproject) Authors.");
            ::LogInfo("Portions Copyright (c) 2015-2021 by Jonathan Naylor, G4KLX and others");

            ::LogInfoEx(LOG_SDR, "DSP is performing initialization and warmup");
            setup();

            ::LogInfoEx(LOG_SDR, "DSP is up and running");
            while (!g_killed) {
                loop();
                ::usleep(1);
            }
        }

        if (g_signal == 2)
            ::LogInfoEx(LOG_SDR, "Exited on receipt of SIGINT");

        if (g_signal == 15)
            ::LogInfoEx(LOG_SDR, "Exited on receipt of SIGTERM");

        if (g_signal == 1)
            ::LogInfoEx(LOG_SDR, "Restarting on receipt of SIGHUP");
    } while (g_signal == 1);

    ::LogInfoEx(LOG_SDR, "DSP is shutting down");

    if (m_serialPort != nullptr) {
        m_serialPort->close();
        delete m_serialPort;
    }

    ::LogFinalise();
    return 0;
}
