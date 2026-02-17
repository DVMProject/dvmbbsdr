# SPDX-License-Identifier: GPL-2.0-only
#/*
# * Digital Voice Modem - Baseband SDR
# * GPLv2 Open Source. Use is subject to license terms.
# * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
# *
# *  Copyright (C) 2024,2025 Bryan Biedenkapp, N2PLL
# *
# */
file(GLOB dvmsdr_SRC
    # DMR module
    "src/sdr/dmr/*.h"
    "src/sdr/dmr/*.cpp"

    # P25 module
    "src/sdr/p25/*.h"
    "src/sdr/p25/*.cpp"

    # NXDN module
    "src/sdr/nxdn/*.h"
    "src/sdr/nxdn/*.cpp"

    # Core
    "src/sdr/modem/*.h"
    "src/sdr/modem/*.cpp"
    "src/sdr/modem/port/*.h"
    "src/sdr/modem/port/*.cpp"
    "src/sdr/*.h"
    "src/sdr/*.cpp"
)
