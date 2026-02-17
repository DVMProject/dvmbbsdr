# SPDX-License-Identifier: GPL-2.0-only
#/*
# * Digital Voice Modem - CMake Build System
# * GPLv2 Open Source. Use is subject to license terms.
# * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
# *
# *  Copyright (C) 2024 Bryan Biedenkapp, N2PLL
# *
# */

# Debug compilation features/options (these should not be enabled for production!)
option(DEBUG_CRC_ADD "" off)
option(DEBUG_CRC_CHECK "" off)
option(DEBUG_RINGBUFFER "" off)

if (DEBUG_CRC_ADD)
    message(CHECK_START "Common CRC Add Debug")
    add_definitions(-DDEBUG_CRC_ADD)
endif (DEBUG_CRC_ADD)
if (DEBUG_CRC_CHECK)
    message(CHECK_START "Common CRC Check Debug")
    add_definitions(-DDEBUG_CRC_CHECK)
endif (DEBUG_CRC_CHECK)
if (DEBUG_RINGBUFFER)
    message(CHECK_START "Ringbuffer Debug")
    add_definitions(-DDEBUG_RINGBUFFER)
endif (DEBUG_RINGBUFFER)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(Threads REQUIRED)

# Check if platform-specific functions exist
include(CheckCXXSymbolExists)
check_cxx_symbol_exists(sendmsg sys/socket.h HAVE_SENDMSG)
check_cxx_symbol_exists(sendmmsg sys/socket.h HAVE_SENDMMSG)

if (HAVE_SENDMSG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAVE_SENDMSG=1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_SENDMSG=1")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DHAVE_SENDMSG=1")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DHAVE_SENDMSG=1")
endif (HAVE_SENDMSG)
if (HAVE_SENDMMSG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAVE_SENDMMSG=1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_SENDMMSG=1")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DHAVE_SENDMMSG=1")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DHAVE_SENDMMSG=1")
endif (HAVE_SENDMMSG)
