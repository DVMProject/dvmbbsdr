# SPDX-License-Identifier: GPL-2.0-only
#/*
# * Digital Voice Modem - CMake Build System
# * GPLv2 Open Source. Use is subject to license terms.
# * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
# *
# *  Copyright (C) 2024 Bryan Biedenkapp, N2PLL
# *
# */
include(src/CompilerOptions.cmake)

#
## common
#
include(src/common/CMakeLists.txt)
add_library(common STATIC ${common_SRC} ${common_INCLUDE})
target_link_libraries(common PRIVATE ${OPENSSL_LIBRARIES} ${LIBDW_LIBRARY} Threads::Threads util)
target_include_directories(common PRIVATE ${OPENSSL_INCLUDE_DIR} ${LIBDW_INCLUDE_DIR} src src/common)

#
## dvmbbsdr
#
include(src/sdr/CMakeLists.txt)
add_executable(dvmbbsdr ${common_INCLUDE} ${dvmsdr_SRC})

target_link_libraries(dvmbbsdr PRIVATE common ${OPENSSL_LIBRARIES} ${LIBDW_LIBRARY} 
    ${Boost_LIBRARIES} 
    ${GNURADIO_PMT_LIBRARIES} 
    ${GNURADIO_RUNTIME_LIBRARIES} 
    ${GNURADIO_FILTER_LIBRARIES} 
    ${GNURADIO_DIGITAL_LIBRARIES} 
    ${GNURADIO_ANALOG_LIBRARIES} 
    ${GNURADIO_AUDIO_LIBRARIES} 
    ${GNURADIO_ZEROMQ_LIBRARIES}
    ${GNURADIO_UHD_LIBRARIES} 
    ${UHD_LIBRARIES} 
    ${GNURADIO_BLOCKS_LIBRARIES} 
    ${GNURADIO_OSMOSDR_LIBRARIES} 
    gnuradio::gnuradio-analog 
    gnuradio::gnuradio-blocks 
    gnuradio::gnuradio-digital 
    gnuradio::gnuradio-filter 
    gnuradio::gnuradio-pmt 
    Threads::Threads 
    util)

if(TARGET gnuradio::gnuradio-zeromq)
    target_link_libraries(dvmbbsdr PRIVATE gnuradio::gnuradio-zeromq)
endif()

if(GNURADIO_ZEROMQ_LIBRARIES OR TARGET gnuradio::gnuradio-zeromq)
    target_link_libraries(dvmbbsdr PRIVATE zmq)
endif()

target_include_directories(dvmbbsdr PRIVATE ${OPENSSL_INCLUDE_DIR} ${LIBDW_INCLUDE_DIR} 
    ${Boost_INCLUDE_DIRS} 
    ${GNURADIO_RUNTIME_INCLUDE_DIRS} 
    ${GNURADIO_ZEROMQ_INCLUDE_DIRS}
    ${GNURADIO_OSMOSDR_INCLUDE_DIRS} 
    src 
    src/sdr)

set(CPACK_SET_DESTDIR true)
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local")

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "dvmbbsdr")
set(CPACK_DEBIAN_PACKAGE_NAME "dvmbbsdr")

set(CPACK_PACKAGE_VENDOR "DVMProject")

set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "DVMProject Authors")
set(CPACK_DEBIAN_PACKAGE_VERSION "R05A")
set(CPACK_DEBIAN_PACKAGE_RELEASE "0")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/dvmproject")

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/postrm")

set(CPACK_DEBIAN_FILE_NAME ${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_DEBIAN_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_RELEASE}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}.deb)

include(CPack)
