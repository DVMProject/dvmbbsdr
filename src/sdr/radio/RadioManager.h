// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Baseband SDR RF Runtime
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * Copyright (C) 2026 Bryan Biedenkapp, N2PLL
 */
/**
 * @file RadioManager.h
 * @ingroup modem_fw
 * @file RadioManager.cpp
 * @ingroup modem_fw
 *
 * @brief Runtime SDR device and channel manager.
 *
 * This manager owns GNU Radio flowgraphs and provides a queue bridge between
 * virtual modem instances and SDR hardware channels.
 */
#if !defined(__RADIO_MANAGER_H__)
#define __RADIO_MANAGER_H__

#include "Defines.h"
#include "common/yaml/Yaml.h"

#include <cstddef>
#include <cstdint>

namespace radio
{

}

#endif // __RADIO_MANAGER_H__
