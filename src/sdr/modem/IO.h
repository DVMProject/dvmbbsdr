// SPDX-License-Identifier: GPL-2.0-only
/*
 * Digital Voice Modem - Modem Firmware
 * GPLv2 Open Source. Use is subject to license terms.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 *  Copyright (C) 2015,2016,2017 Jonathan Naylor, G4KLX
 *  Copyright (C) 2017-2024 Bryan Biedenkapp, N2PLL
 *
 */
/**
 * @file IO.h
 * @ingroup modem_fw
 * @file IO.cpp
 * @ingroup modem_fw
 * @file IODue.cpp
 * @ingroup modem_fw
 * @file IOSTM.cpp
 * @ingroup modem_fw
 */
#if !defined(__IO_H__)
#define __IO_H__

#include "Defines.h"
#include "modem/SerialPort.h"
#include "SampleBuffer.h"
#include "RSSIBuffer.h"

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

/**
 * @addtogroup hotspot_fw
 * @{
 */

#define DEFAULT_FREQUENCY       433075000

/** Band Tables */
/** 136 - 174 mhz */
#define VHF_MIN                 136000000
#define VHF_MAX                 174000000

/** 216 - 225 mhz */
#define VHF_220_MIN             216000000
#define VHF_220_MAX             225000000

/** 380 - 431mhz */
#define UHF_380_MIN             380000000
#define UHF_380_MAX             431000000

/** 431 - 450mhz */
#define UHF_1_MIN               431000000
#define UHF_1_MAX               470000000

/** 450 - 470mhz */
#define UHF_2_MIN               450000000
#define UHF_2_MAX               470000000

/** 470 - 520mhz (T-band) */
#define UHF_T_MIN               470000000
#define UHF_T_MAX               520000000

/** 842 - 900mhz */
#define UHF_800_MIN             842000000
#define UHF_800_MAX             900000000

/** 900 - 950mhz */
#define UHF_900_MIN             900000000
#define UHF_900_MAX             950000000

/** @} */

namespace modem
{
    // ---------------------------------------------------------------------------
    //  Class Declaration
    // ---------------------------------------------------------------------------

    /**
     * @brief Implements the input/output data path with the radio air interface.
     * @ingroup modem_fw
     */
    class DSP_FW_API IO {
    public:
        /**
         * @brief Initializes a new instance of the IO class.
         */
        IO(modem::Modem* modem);
        /**
         * @brief Finalizes a new instance of the IO class.
         */
        ~IO();

        /**
         * @brief Starts air interface sampler.
         */
        void start();

        /**
         * @brief Process samples from air interface.
         */
        void process();

        /**
         * @brief Write samples to air interface.
         * @param mode 
         * @param samples Samples to write.
         * @param length Length of samples buffer.
         * @param control 
         */
        void write(DVM_STATE mode, q15_t* samples, uint16_t length, const uint8_t* control = NULL);

        /**
         * @brief Helper to get how much space the transmit ring buffer has for samples.
         * @returns uint16_t Amount of space in the transmit ring buffer for samples.
         */
        uint16_t getSpace() const;

        /**
         * @brief 
         * @param dcd 
         */
        void setDecode(bool dcd);
        /**
         * @brief 
         * @param detect 
         */
        void setADCDetection(bool detect);
        /**
         * @brief Helper to set the modem air interface state.
         */
        void setMode();
        /**
         * @brief Helper to assert or deassert radio PTT.
         */
        void setTransmit();

        /**
         * @brief Hardware interrupt handler.
         */
        void interrupt();

        /**
         * @brief Sets various air interface parameters.
         * @param rxInvert Flag indicating the Rx polarity should be inverted.
         * @param txInvert Flag indicating the Tx polarity should be inverted.
         * @param pttInvert Flag indicating the PTT polarity should be inverted.
         * @param rxLevel Rx Level.
         * @param cwIdTXLevel CWID Transmit Level.
         * @param dmrTXLevel DMR Transmit Level.
         * @param p25TXLevel P25 Transmit Level.
         * @param nxdnTXLevel NXDN Transmit Level.
         * @param txDCOffset Tx DC offset parameter.
         * @param rxDCOffset Rx DC offset parameter.
         */
        void setParameters(bool rxInvert, bool txInvert, bool pttInvert, uint8_t rxLevel, uint8_t cwIdTXLevel, uint8_t dmrTXLevel,
                        uint8_t p25TXLevel, uint8_t nxdnTXLevel, uint16_t txDCOffset, uint16_t rxDCOffset);
        /**
         * @brief Sets the software Rx sample level.
         * @param rxLevel Rx Level.
         */
        void setRXLevel(uint8_t rxLevel);
        /**
         * @brief Sets the RF parameters.
         * @param rxFreq Receive Frequency (hz).
         * @param txFreq Transmit Frequency (hz).
         * @param rfPower RF Power Level.
         * @returns uint8_t Reason code.
         */
        uint8_t setRFParams(uint32_t rxFreq, uint32_t txFreq, uint8_t rfPower);
        /**
         * @brief Sets the RF adjustment parameters.
         * @param dmrDiscBWAdj DMR Discriminator Bandwidth Adjust.
         * @param p25DiscBWAdj P25 Discriminator Bandwidth Adjust.
         * @param nxdnDiscBWAdj NXDN Discriminator Bandwidth Adjust.
         * @param dmrPostBWAdj DMR Post Bandwidth Adjust.
         * @param p25PostBWAdj P25 Post Bandwidth Adjust.
         * @param nxdnPostBWAdj NXDN Post Bandwidth Adjust.
         */
        void setRFAdjust(int8_t dmrDiscBWAdj, int8_t p25DiscBWAdj, int8_t nxdnDiscBWAdj, int8_t dmrPostBWAdj, int8_t p25PostBWAdj, int8_t nxdnPostBWAdj);
        /**
         * @brief Sets the RF AFC adjustment parameters.
         * @param afcEnable Flag indicating the Automatic Frequency Correction is enabled.
         * @param afcKI 
         * @param afcKP 
         * @param afcRange 
         */
        void setAFCParams(bool afcEnable, uint8_t afcKI, uint8_t afcKP, uint8_t afcRange);

        /**
         * @brief Helper to get the state of the ADC and DAC overflow flags.
         * @param[out] adcOverflow 
         * @param[out] dacOverflow 
         */
        void getOverflow(bool& adcOverflow, bool& dacOverflow);

        /**
         * @brief Flag indicating the TX ring buffer has overflowed.
         * @returns bool Flag indicating the TX ring buffer has overflowed.
         */
        bool hasTXOverflow();
        /**
         * @brief Flag indicating the RX ring buffer has overflowed.
         * @returns bool Flag indicating the RX ring buffer has overflowed.
         */
        bool hasRXOverflow();

        /**
         * @brief Flag indicating the air interface is locked out from transmitting.
         * @returns bool Flag indicating the air interface is locked out from transmitting.
         */
        bool hasLockout() const;

        /**
         * @brief 
         */
        void resetWatchdog();
        /**
         * @brief 
         * @returns uint32_t 
         */
        uint32_t getWatchdog();

        /**
         * @brief Gets the CPU type the firmware is running on.
         * @returns uint8_t 
         */
        uint8_t getCPU() const;

        /**
         * @brief Gets the unique identifier for the air interface.
         * @param buffer 
         */
        void getUDID(uint8_t* buffer);

        /**
         * @brief 
         */
        void resetMCU();

    private:
        friend class modem::Modem;
        modem::Modem* m_modem;

        bool m_started;

        SampleBuffer m_rxBuffer;
        SampleBuffer m_txBuffer;
        RSSIBuffer m_rssiBuffer;

        arm_fir_instance_q15 m_rrc_0_2_Filter;
        arm_fir_instance_q15 m_boxcar_5_Filter;

        arm_biquad_casd_df1_inst_q31 m_dcFilter;

        q15_t m_rrc_0_2_State[70U];      // NoTaps + BlockSize - 1, 42 + 20 - 1 plus some spare
        q15_t m_boxcar_5_State[30U];     // NoTaps + BlockSize - 1, 6 + 20 - 1 plus some spare

    #if NXDN_BOXCAR_FILTER
        arm_fir_instance_q15 m_boxcar_10_Filter;

        q15_t m_boxcar_10_State[40U];   // NoTaps + BlockSize - 1, 10 + 20 - 1 plus some spare
    #else
        arm_fir_instance_q15 m_nxdn_0_2_Filter;
        arm_fir_instance_q15 m_nxdn_ISinc_Filter;
        
        q15_t m_nxdn_0_2_State[110U];   // NoTaps + BlockSize - 1, 82 + 20 - 1 plus some spare
        q15_t m_nxdn_ISinc_State[60U];  // NoTaps + BlockSize - 1, 32 + 20 - 1 plus some spare
    #endif

        q31_t m_dcState[4];

        bool m_pttInvert;
        q15_t m_rxLevel;
        bool m_rxInvert;
        q15_t m_cwIdTXLevel;
        q15_t m_dmrTXLevel;
        q15_t m_p25TXLevel;
        q15_t m_nxdnTXLevel;

        uint16_t m_rxDCOffset;
        uint16_t m_txDCOffset;

        uint32_t m_ledCount;
        bool m_ledValue;

        bool m_detect;

        uint16_t m_adcOverflow;
        uint16_t m_dacOverflow;

        volatile uint32_t m_watchdog;

        bool m_lockout;

        uint32_t m_rxFrequency;
        uint32_t m_txFrequency;
        uint8_t m_rfPower;

        int8_t m_dmrDiscBWAdj;
        int8_t m_p25DiscBWAdj;
        int8_t m_nxdnDiscBWAdj;
        int8_t m_dmrPostBWAdj;
        int8_t m_p25PostBWAdj;
        int8_t m_nxdnPostBWAdj;

        bool m_afcEnable;
        uint8_t m_afcKI;
        uint8_t m_afcKP;
        uint8_t m_afcRange;

        // Hardware specific routines
        /**
         * @brief Starts hardware interrupts.
         */
        void startInt();

        /**
         * @brief 
         */
        bool getCOSInt();

        /**
         * @brief 
         * @param on 
         */
        void setLEDInt(bool on);
        /**
         * @brief 
         * @param on 
         */
        void setPTTInt(bool on);
        /**
         * @brief 
         * @param on 
         */
        void setCOSInt(bool on);

        /**
         * @brief 
         * @param on 
         */
        void setDMRInt(bool on);
        /**
         * @brief 
         * @param on 
         */
        void setP25Int(bool on);
        /**
         * @brief 
         * @param on 
         */
        void setNXDNInt(bool on);

        /**
         * @brief 
         * @param dly 
         */
        void delayInt(unsigned int dly);

        /**
         * @brief 
         * @param arg 
         * @returns void* 
         */
        static void* txThreadHelper(void* arg);
        /**
         * @brief 
         * @param interruptRx 
         */
        void interruptRx();
        /**
         * @brief 
         * @param arg 
         * @returns void* 
         */
        static void* rxThreadHelper(void* arg);
    };
} // namespace modem

#endif // __IO_H__
