# SPDX-License-Identifier: GPL-2.0-only
#/*
# * Digital Voice Modem - Common Library
# * GPLv2 Open Source. Use is subject to license terms.
# * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
# *
# *  Copyright (C) 2024,2025 Bryan Biedenkapp, N2PLL
# *
# */
file(GLOB common_SRC
    # Core
    "src/common/edac/*.cpp"
    "src/common/network/*.cpp"
    "src/common/network/udp/*.cpp"
    "src/common/yaml/*.cpp"
    "src/common/*.cpp"
)

file(GLOB common_INCLUDE
    # Core
    "src/common/concurrent/*.h"
    "src/common/edac/*.h"
    "src/common/json/*.h"
    "src/common/network/*.h"
    "src/common/network/udp/*.h"
    "src/common/backtrace/*.h"
    "src/common/yaml/*.h"
    "src/common/*.h"
)
