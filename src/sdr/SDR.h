// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Modem Firmware
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 *
 */
/**
 * @file SDR.h
 * @ingroup modem_fw
 * @file SDR.cpp
 * @ingroup modem_fw
 */
#if !defined(__SDR_H__)
#define __SDR_H__

#include "Defines.h"
#include "common/yaml/Yaml.h"

#include <string>
#include <mutex>

// ---------------------------------------------------------------------------
//  Class Declaration
// ---------------------------------------------------------------------------

/**
 * @brief This class implements the core service logic.
 * @ingroup modem_fw
 */
class HOST_SW_API SDR {
public:
    /**
     * @brief Initializes a new instance of the SDR class.
     * @param confFile Full-path to the configuration file.
     */
    SDR(const std::string& confFile);
    /**
     * @brief Finalizes a instance of the SDR class.
     */
    ~SDR();

    /**
     * @brief Executes the main host processing loop.
     * @returns int Zero if successful, otherwise error occurred.
     */
    int run();

private:
    const std::string& m_confFile;
    yaml::Node m_conf;

    static bool s_running;
    bool m_trace;
    bool m_debug;

    /**
     * @brief Reads basic configuration parameters from the INI.
     * @returns bool True, if configuration was read successfully, otherwise false.
     */
    bool readParams();
};

#endif // __SDR_H__
