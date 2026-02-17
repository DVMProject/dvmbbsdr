# SPDX-License-Identifier: GPL-2.0-only
#/*
# * Digital Voice Modem - CMake Build System
# * GPLv2 Open Source. Use is subject to license terms.
# * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
# *
# *  Copyright (C) 2022,2024,2025 Bryan Biedenkapp, N2PLL
# *  Copyright (C) 2022 Natalie Moore
# *
# */
cmake_minimum_required(VERSION 3.16.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS on) 

set(ENABLE_LIBDW_SUPPORT ON)

set(CMAKE_C_COMPILER /usr/bin/gcc CACHE STRING "C compiler")
set(CMAKE_CXX_COMPILER /usr/bin/g++ CACHE STRING "C++ compiler")

# detect and set architecture
set(ARCH amd64)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH amd64)
    set(CMAKE_SYSTEM_PROCESSOR amd64)
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE amd64)
else()
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "")
        execute_process(COMMAND uname -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
        string(REGEX REPLACE "\n$" "" CMAKE_SYSTEM_PROCESSOR "${CMAKE_SYSTEM_PROCESSOR}")
        set(ARCH ${CMAKE_SYSTEM_PROCESSOR})
    else()
        set(ARCH ${CMAKE_SYSTEM_PROCESSOR})
    endif (CMAKE_SYSTEM_PROCESSOR MATCHES "")
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
message(CHECK_START "Detect Host Architecture - ${ARCH}")

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE amd64)

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Standard CMake options
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY .)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)
message(CHECK_START "Build Type is ${CMAKE_BUILD_TYPE}")
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -Wno-overloaded-virtual -std=c++14")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g -O0 -Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O0 -Wall -Wno-overloaded-virtual -std=c++14")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3 -Wall -s")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -Wno-overloaded-virtual -std=c++14 -s")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g -O3 -Wall -s")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3 -Wall -Wno-overloaded-virtual -std=c++14 -s")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3 -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -Wno-overloaded-virtual -std=c++14")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g -O3 -Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3 -Wall -Wno-overloaded-virtual -std=c++14")
endif()

set(CMAKE_INSTALL_PREFIX "/usr/local")

#
# Library Inclusions
#

# elfutils (libdw-dev)
if (ENABLE_LIBDW_SUPPORT)
    find_path(LIBDW_INCLUDE_DIR NAMES elfutils/libdw.h elfutils/libdwfl.h HINTS /usr /usr/local PATH_SUFFIXES include)

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Libdw DEFAULT_MSG LIBDW_INCLUDE_DIR)

    if (LIBDW_FOUND)
        message("-- libdw (libdw-dev) found, detailed backtrace support enabled")
        add_definitions(-DBACKWARD_HAS_DW=1)
        set(LIBDW_LIBRARY "dw")
    else()
        message("-- libdw (libdw-dev) not found, simple backtrace only")
        add_definitions(-DBACKWARD_HAS_DW=0 -DBACKWARD_HAS_BACKTRACE_SYMBOL=1)
        set(LIBDW_INCLUDE_DIR "")
        set(LIBDW_LIBRARY "")
    endif (LIBDW_FOUND)

    mark_as_advanced(LIBDW_INCLUDE_DIR LIBDW_LIBRARY)
else()
    message("-- libdw (libdw-dev) disabled, simple backtrace only")
    add_definitions(-DBACKWARD_HAS_DW=0 -DBACKWARD_HAS_BACKTRACE_SYMBOL=1)
    set(LIBDW_INCLUDE_DIR "")
    set(LIBDW_LIBRARY "")
endif (ENABLE_LIBDW_SUPPORT)

#
# TODO TODO TODO
#

#
# Set GIT_VER compiler directive
#
set(GIT_VER "")
set(GIT_VER_HASH "")
execute_process(COMMAND git describe --abbrev=8 --dirty --always WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} OUTPUT_VARIABLE GIT_VER OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND git describe --abbrev=8 --always WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} OUTPUT_VARIABLE GIT_VER_HASH OUTPUT_STRIP_TRAILING_WHITESPACE)

add_definitions(-D__GIT_VER__="${GIT_VER}")
add_definitions(-D__GIT_VER_HASH__="${GIT_VER_HASH}")

project(dvmbbsdr)
include(src/CMakeLists.txt)

#
# Standard dvmbbsdr install
#
install(TARGETS dvmbbsdr DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(FILES configs/config-sdr.example.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/etc)
install(CODE "execute_process(COMMAND bash \"-c\" \"sed -i 's/filePath: ./filePath: \\\\/var\\\\/log\\\\//' /usr/local/etc/config.example.yml\")")

#
# Helper target to force strip binaries.
#
if (NOT TARGET strip)
    add_custom_target(strip
        COMMAND strip -s dvmbbsdr)
endif (NOT TARGET strip)

#/*
#** bryanb: Please do not change the following section unless adding or removing paths that need to be part of a build
#** these sections are maintained for internal use.
#*/

#
# Custom make target to perform a tarball packaging. This will ultimately contain the same type of pathing
# the non-standard legacy install to "/opt/dvm" does.
# 
if (NOT TARGET tarball)
    set(CMAKE_INSTALL_PREFIX_TARBALL "tar_build")
    add_custom_target(tarball
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX_TARBALL}
        COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX_TARBALL}/dvm/bin
        COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX_TARBALL}/dvm/log
        COMMAND touch ${CMAKE_INSTALL_PREFIX_TARBALL}/dvm/log/INCLUDE_DIRECTORY
        COMMAND cp -v dvmbbsdr ${CMAKE_INSTALL_PREFIX_TARBALL}/dvm/bin
        COMMAND cp -v ${CMAKE_SOURCE_DIR}/configs/*.yml ${CMAKE_INSTALL_PREFIX_TARBALL}/dvm
        COMMAND cd ${CMAKE_INSTALL_PREFIX_TARBALL} && tar czvf ../dvmhost_${CPACK_DEBIAN_PACKAGE_VERSION}_${ARCH}.tar.gz *
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX_TARBALL})
endif (NOT TARGET tarball)

#
# Custom make target to perform non-standard legacy install to "/opt/dvm". This is meant
# to retain backward compatibility with deployment scripts and other tools that work with "/opt/dvm"
# instead of system paths.
#
# This is inherited logic from the old DVMHost 2.0 Makefile.
#
set(CMAKE_LEGACY_INSTALL_PREFIX "/opt/dvm")
add_custom_target(old_install
    COMMAND mkdir -p ${CMAKE_LEGACY_INSTALL_PREFIX}
    COMMAND mkdir -p ${CMAKE_LEGACY_INSTALL_PREFIX}/bin
    COMMAND mkdir -p ${CMAKE_LEGACY_INSTALL_PREFIX}/log
    COMMAND mkdir -p ${CMAKE_LEGACY_INSTALL_PREFIX}/schema
    COMMAND install -m 755 dvmbbsdr ${CMAKE_LEGACY_INSTALL_PREFIX}/bin
    COMMAND install -m 644 ${CMAKE_SOURCE_DIR}/configs/config-sdr.example.yml ${CMAKE_LEGACY_INSTALL_PREFIX}/config-sdr.example.yml)
