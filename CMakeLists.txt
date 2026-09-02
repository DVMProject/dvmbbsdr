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
cmake_minimum_required(VERSION 3.12.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS on) 

set(ENABLE_LIBDW_SUPPORT ON)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_C_COMPILER /usr/bin/gcc CACHE STRING "C compiler")
set(CMAKE_CXX_COMPILER /usr/bin/g++ CACHE STRING "C++ compiler")

set(CMAKE_CXX_STANDARD 17)

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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -Wno-overloaded-virtual -std=c++17")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g -O0 -Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O0 -Wall -Wno-overloaded-virtual -std=c++17")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3 -Wall -s")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -Wno-overloaded-virtual -std=c++17 -s")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g -O3 -Wall -s")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3 -Wall -Wno-overloaded-virtual -std=c++17 -s")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3 -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -Wno-overloaded-virtual -std=c++17")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g -O3 -Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3 -Wall -Wno-overloaded-virtual -std=c++17")
endif()

set(CMAKE_INSTALL_PREFIX "/usr/local")

project(dvmbbsdr)

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

# libssl-dev
message(STATUS "OpenSSL root dir: ${OPENSSL_ROOT_DIR}")
find_package(OpenSSL REQUIRED)
if (OpenSSL_FOUND)
    message(STATUS "OpenSSL include dir: ${OPENSSL_INCLUDE_DIR}")
    message(STATUS "OpenSSL libraries: ${OPENSSL_LIBRARIES}")
    message(CHECK_START "Enable SSL support")
    message(CHECK_PASS "yes")

    add_definitions(-DENABLE_SSL)
    include_directories("${OPENSSL_INCLUDE_DIR}")
else (OpenSSL_FOUND)
    message(STATUS "OpenSSL not found, SSL support disabled")
    message(CHECK_START "Enable SSL support")
    message(CHECK_PASS "no")
endif (OpenSSL_FOUND)

# GnuRadio (gnuradio-dev)
include(FindPkgConfig)
find_package(Gnuradio 3.10 REQUIRED)
include(GrVersion)

set(GR_REQUIRED_COMPONENTS RUNTIME ANALOG BLOCKS DIGITAL FILTER FFT PMT)
find_package(Gnuradio REQUIRED COMPONENTS runtime analog blocks digital filter fft)
if(Gnuradio_VERSION VERSION_LESS "3.8")
    find_package(Volk)
endif()

# Optional GNU Radio ZeroMQ blocks used for headless RX FFT/export taps.
find_package(Gnuradio QUIET COMPONENTS zeromq)
if(GNURADIO_ZEROMQ_LIBRARIES OR TARGET gnuradio::gnuradio-zeromq)
    add_definitions(-DHAS_GNURADIO_ZEROMQ=1)
    message(STATUS "GnuRadio ZeroMQ support: enabled")
else()
    message(STATUS "GnuRadio ZeroMQ support: disabled")
endif()

if(NOT CMAKE_MODULES_DIR)
  set(CMAKE_MODULES_DIR lib${LIB_SUFFIX}/cmake)
endif(NOT CMAKE_MODULES_DIR)

if(Gnuradio_VERSION VERSION_LESS "3.8")
    find_package(Volk)
endif()

if(NOT Gnuradio_FOUND)
    message(FATAL_ERROR "GnuRadio Runtime required to compile gqrx")
endif()

set(GR_RUNTIME_DIR      bin)
set(GR_LIBRARY_DIR      lib${LIB_SUFFIX})
set(GR_INCLUDE_DIR      include/howto)
set(GR_DATA_DIR         share)
set(GR_PKG_DATA_DIR     ${GR_DATA_DIR}/${CMAKE_PROJECT_NAME})
set(GR_DOC_DIR          ${GR_DATA_DIR}/doc)
set(GR_PKG_DOC_DIR      ${GR_DOC_DIR}/${CMAKE_PROJECT_NAME})
set(GR_CONF_DIR         etc)
set(GR_PKG_CONF_DIR     ${GR_CONF_DIR}/${CMAKE_PROJECT_NAME}/conf.d)
set(GR_LIBEXEC_DIR      libexec)
set(GR_PKG_LIBEXEC_DIR  ${GR_LIBEXEC_DIR}/${CMAKE_PROJECT_NAME})
set(GRC_BLOCKS_DIR      ${GR_PKG_DATA_DIR}/grc/blocks)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

math(EXPR GNURADIO_VERSION "(${Gnuradio_VERSION_MAJOR} / 10) << 20 | (${Gnuradio_VERSION_MAJOR} % 10) << 16 | (${Gnuradio_VERSION_MINOR} / 10) << 12 | (${Gnuradio_VERSION_MINOR} % 10) <<  8 | (${Gnuradio_VERSION_PATCH} / 10) <<  4 | (${Gnuradio_VERSION_PATCH} % 10) <<  0   ")
message(STATUS "GnuRadio Version: " ${GNURADIO_VERSION})

find_package(GnuradioUHD)
find_package(GnuradioOsmosdr)
find_package(LibHackRF)
find_package(LibUHD)
find_package(SoapySDR)

include(GrPlatform)

# boost
MESSAGE(STATUS "Configuring Boost C++ Libraries...")

# Although not required on my system, some users have linking issues without
SET(BOOST_REQUIRED_COMPONENTS
    thread
    program_options
    filesystem
    log_setup
    log
    regex
    random
)

if(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

set(Boost_ADDITIONAL_VERSIONS
    "1.35.0" "1.35" "1.36.0" "1.36" "1.37.0" "1.37" "1.38.0" "1.38" "1.39.0" "1.39"
    "1.40.0" "1.40" "1.41.0" "1.41" "1.42.0" "1.42" "1.43.0" "1.43" "1.44.0" "1.44"
    "1.45.0" "1.45" "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
    "1.70.0" "1.70" "1.71.0" "1.71" "1.72.0" "1.72" "1.73.0" "1.73" "1.74.0" "1.74"
)

find_package(Boost COMPONENTS ${BOOST_REQUIRED_COMPONENTS} REQUIRED)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost required to build " ${CMAKE_PROJECT_NAME})
endif()

ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)
ADD_DEFINITIONS(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
if(NOT EXISTS /usr/include/spdlog/fmt/bundled)
  ADD_DEFINITIONS(-DSPDLOG_FMT_EXTERNAL)
endif()
add_definitions(-DGNURADIO_VERSION=${GNURADIO_VERSION})
message(STATUS "Gnuradio Version is: " ${Gnuradio_VERSION})

include_directories(${Boost_INCLUDE_DIRS} ${GNURADIO_RUNTIME_INCLUDE_DIRS} ${GNURADIO_OSMOSDR_INCLUDE_DIRS} ${GNURADIO_ZEROMQ_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/include ${OPENSSL_INCLUDE_DIR} ${SOAPY_SDR_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${GNURADIO_RUNTIME_LIBRARY_DIRS} ${GROSMOSDR_LIBRARIES_DIRS} ${GNURADIO_UHD_LIBRARIES_DIRS} ${UHD_LIBRARIES_DIRS} ${OPENSSL_ROOT_DIR}/lib ${SOAPY_SDR_LIBRARIES_DIR})

#
# Set GIT_VER compiler directive
#
set(GIT_VER "")
set(GIT_VER_HASH "")
execute_process(COMMAND git describe --abbrev=8 --dirty --always WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} OUTPUT_VARIABLE GIT_VER OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND git describe --abbrev=8 --always WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} OUTPUT_VARIABLE GIT_VER_HASH OUTPUT_STRIP_TRAILING_WHITESPACE)

add_definitions(-D__GIT_VER__="${GIT_VER}")
add_definitions(-D__GIT_VER_HASH__="${GIT_VER_HASH}")

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
