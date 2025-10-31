//#############################################################################
//
// FILE:   clllc_settings.h
//
// TITLE: This is the settings.h file, which contains all the project level
//        settings, in case of powerSUITE , powerSUITE related settings are
//        in this file and the other settings are located in 
//        clllc_user_settings.h.
//        The User section is not over-written by powerSUITE
//
//#############################################################################
// Copyright (C) {2019} Texas Instruments Incorporated - http://www.ti.com/
// * ALL RIGHTS RESERVED*
//#############################################################################
#ifndef _CLC_PROJSETTINGS_H
#define _CLC_PROJSETTINGS_H

#ifdef __cplusplus

extern "C" {
#endif

//#############################################################################
//
// Includes
//
//#############################################################################
#include <stdint.h>

//#############################################################################
//
// Macro Definitions
//
//#############################################################################
#ifndef C2000_IEEE754_TYPES
    #define C2000_IEEE754_TYPES
    #ifdef __TI_EABI__
        typedef float         float32_t;
        typedef double        float64_t;
    #else // TI COFF
        typedef float         float32_t;
        typedef long double   float64_t;
    #endif // __TI_EABI__
#endif // C2000_IEEE754_TYPES

// Device Related Defines
//
#define CLC_CPU_SYS_CLK_FREQ_HZ       ((float32_t)200*1000000)
#define CLC_PWMSYSCLOCK_FREQ_HZ       ((float32_t)200*1000000)
#define CLC_ECAPSYSCLOCK_FREQ_HZ      ((float32_t)200*1000000)
// Project Options

// CONTROL MODE , voltage or current
#define CLC_VOLTAGE_MODE 1
#define CLC_CURRENT_MODE 2

// PROTECTION
#define CLC_PROTECT_ENB 1
#define CLC_PROTECT_DIS 0

// BUILD
#define CLC_OPEN_LOOP_BUILD 1
#define CLC_CLOSED_LOOP_BUILD 2

//
// TEST, (which side is output depends on power flow)
// 0 ->  Test with Res load at the output
// 1 ->  Test with Res Load and Voltage source connected at output
#define CLC_SEC_CONECT_IN_BATTY_EMULATE_MD 0
#define CLC_TEST_SET_RES_LOAD 0
#define CLC_TEST_SET_BATTERY 1


//
// CORE running the control loop
// 1 -> C28x
// 2 -> CLA
#ifndef C28x_CORE
#define C28x_CORE 1
#define CLA_CORE 2
#endif

//
// SFRA Options
// 0 -> disabled
// 1 -> Current
// 2 -> Voltage
#define CLC_SFRA_DISABLED 0
#define CLC_SFRA_CURRENT 1
#define CLC_SFRA_VOLTAGE 2

//
// SFRA injection amplitude, use higher injection in open loop  because plant
// response is low
#define CLC_SFRA_INJECTION_AMPLITUDE_LEVEL1 0.001
#define CLC_SFRA_INJECTION_AMPLITUDE_LEVEL2 0.01
#define CLC_SFRA_INJECTION_AMPLITUDE_LEVEL3 0.015

//
// CLLLC LAB
// Power Flow Prim -> Sec
// 1 -> Open loop check for PWM drivers,
// 2 -> Open loop check for PWM drivers with protection,
// 3 -> Closed loop check with resistive load, voltage loop,
// 4 -> Closed loop check with resistive load, current loop
// 5 -> Closed loop check with battery emulated, current loop
// Power Flow Sec -> Prim
// 6 -> Open loop check for PWM driver,
// 7 -> Open loop check for PWM driver with protection,
// 8 -> Closed loop voltage with resistive load

#define CLC_LAB 2

#if CLC_LAB == 1
#define CLC_CONTROL_RUNNING_ON 1
#define CLC_POWER_FLOW CLC_PWR_FLOW_PRIM_SEC
#define CLC_INCR_BUILD CLC_OPEN_LOOP_BUILD
#define CLC_TEST_SETUP CLC_TEST_SET_RES_LOAD
#define CLC_PROTECTION CLC_PROTECT_ENB
#endif

#if CLC_LAB == 2
#define CLC_CONTROL_RUNNING_ON 1
#define CLC_POWER_FLOW CLC_PWR_FLOW_PRIM_SEC
#define CLC_INCR_BUILD CLC_CLOSED_LOOP_BUILD
#define CLC_CONTROL_MODE CLC_VOLTAGE_MODE
#define CLC_TEST_SETUP CLC_TEST_SET_RES_LOAD
#define CLC_PROTECTION CLC_PROTECT_ENB
#endif

#if CLC_LAB == 3
#define CLC_CONTROL_RUNNING_ON 1
#define CLC_POWER_FLOW CLC_PWR_FLOW_PRIM_SEC
#define CLC_INCR_BUILD CLC_CLOSED_LOOP_BUILD
#define CLC_CONTROL_MODE CLC_CURRENT_MODE
#define CLC_TEST_SETUP CLC_TEST_SET_BATTERY
#define CLC_PROTECTION CLC_PROTECT_ENB
#endif

#define CLC_ISR1_RUNNING_ON CLC_CONTROL_RUNNING_ON
#define CLC_ISR2_FREQUENCY_HZ ((float32_t)100000)
#define CLC_ISR3_FREQUENCY_HZ ((float32_t)10000)

#define CLC_SFRA_ISR_FREQ_HZ       CLC_ISR2_FREQUENCY_HZ

// Power Stage Related Values
#define CLC_NOMINAL_PWM_SWITCH_FREQ_HZ      ((float32_t)450*1000)
#define CLC_MAX_PWM_SWITCH_FREQ_HZ          ((float32_t)600*1000)
#define CLC_MIN_PWM_SWITCH_FREQ_HZ          ((float32_t)360*1000)

#define CLC_PRIM_PWM_DEADBAND_RED_NS        ((float32_t)150)
#define CLC_PRIM_PWM_DEADBAND_FED_NS        ((float32_t)300)
#define CLC_SEC_PWM_DEADBAND_RED_NS         ((float32_t)150)
#define CLC_SEC_PWM_DEADBAND_FED_NS         ((float32_t)300)

// Control Loop Design
#define CLC_GV1_2P2Z_A1    (float32_t) -1.7284895037
#define CLC_GV1_2P2Z_A2    (float32_t) 0.7284895037
#define CLC_GV1_2P2Z_A3    (float32_t) 0.0000000000
#define CLC_GV1_2P2Z_B0    (float32_t) 4.8280130584
#define CLC_GV1_2P2Z_B1    (float32_t) 0.1493277469
#define CLC_GV1_2P2Z_B2    (float32_t) -4.6786792593
#define CLC_GV1_2P2Z_B3    (float32_t) 0.0000000000

#define CLC_GI1_2P2Z_A1    (float32_t) -1.8277396009
#define CLC_GI1_2P2Z_A2    (float32_t) 0.8277396009
#define CLC_GI1_2P2Z_A3    (float32_t) 0.0000000000
#define CLC_GI1_2P2Z_B0    (float32_t) 1.2500036172
#define CLC_GI1_2P2Z_B1    (float32_t) 0.2153188876
#define CLC_GI1_2P2Z_B2    (float32_t) -1.0346715071
#define CLC_GI1_2P2Z_B3    (float32_t) 0.0000000000

#define CLC_GI2_2P2Z_A1    (float32_t) 0.0341879720
#define CLC_GI2_2P2Z_A2    (float32_t) -0.7668017816
#define CLC_GI2_2P2Z_A3    (float32_t) -0.2673861903
#define CLC_GI2_2P2Z_B0    (float32_t) 1.3436620732
#define CLC_GI2_2P2Z_B1    (float32_t) 0.3459370813
#define CLC_GI2_2P2Z_B2    (float32_t) -0.7200660800
#define CLC_GI2_2P2Z_B3    (float32_t) -0.2790608258

#define CLC_GV2_2P2Z_A1    (float32_t) -0.4829060140
#define CLC_GV2_2P2Z_A2    (float32_t) -0.5170939860
#define CLC_GV2_2P2Z_A3    (float32_t) 0.0000000000
#define CLC_GV2_2P2Z_B0    (float32_t) 1.3436620732
#define CLC_GV2_2P2Z_B1    (float32_t) -0.3488624959
#define CLC_GV2_2P2Z_B2    (float32_t) -0.5396713815
#define CLC_GV2_2P2Z_B3    (float32_t) 0.0000000000

//=============================================================================
// User code settings file
//=============================================================================
#include "clllc_user_settings.h"

#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif //_PROJSETTINGS_H
