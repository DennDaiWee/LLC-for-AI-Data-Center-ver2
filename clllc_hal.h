//#############################################################################
//
// FILE:   clllc_hal.h
//
// TITLE: solution hardware abstraction layer header file
//        This file consists of common variables and functions
//        for a particular hardware board, like functions to read current
//        and voltage signals on the board and functions to setup the
//        basic peripherals of the board
//
//#############################################################################
// $TI Release: TIDM_02002 v2.00.03.00 $
// $Release Date: Fri Feb 25 04:13:41 CST 2022 $
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef CLC_HAL_H
#define CLC_HAL_H

#ifdef __cplusplus

extern "C" {
#endif


//
// the includes
//

#include "inc/hw_types.h"
#include "driverlib.h"
#include "device.h"
#include "clllc_settings.h"
#include "Can_hal.h"

//
// the function prototypes
//
void clc_HAL_setDevice(void);
void clc_HAL_setADC(void);
//void CLC_HAL_setupIprimSenseSignalChain(void);
void clc_HAL_setProtect(void);
void clc_Hal_setSynchrsRectifiAct();
void clc_HAL_setSynchrsRectifiActDebug();
void clc_HAL_setTrigForADC(void);
void clc_HAL_setLED(void);
void clc_HAL_toggleLED1(void);
void clc_HAL_setProfileGPIO(void);
void clc_HAL_setSCI(void);
void clc_HAL_sendCmdOverSci(uint16_t mode, uint16_t voltage_ref);
void clc_HAL_setBdVoutProtect(void);
//
//! \brief Sets up mux for PWM pins based on the mode
//! \param mode  Define the mode of the PWM pins
//!              Mode = 0, All PWMs are zero/disabled
//!              Mode = 1, Prim PWMs are ON, Sec PWMs are OFF
//!              Mode = 2, Prim PWMs are ON, Sec PWMs are ON
//! \return None
//!
//
void clc_Hal_setPWMpins(uint16_t mode);
void clc_Hal_setCLA(void);
void clc_Hal_setPWM(void);
void clc_Hal_setCMPSSHighLowLimit(uint32_t base1, float32_t currentLimit, float32_t currentMaxSense, uint16_t hysteresis,
                                  uint16_t filterClkPrescalar, uint16_t filterSampleWindow, uint16_t filterThreshold);


void clc_Hal_setIPRMtankCmpss_HighLowLim(uint32_t base1, float32_t currentLimit, float32_t currentMaxSense, uint16_t hysteresis,
                                         uint16_t filterClkPrescalar, uint16_t filterSampleWindow, uint16_t filterThreshold);

void clc_Hal_setCmpss_HighLowVoltLim(uint32_t base1, float32_t VoltLimit, float32_t VoltLowLimit, float32_t voltMaxSense,
                                     uint16_t hysteresis, uint16_t filterClkPrescalar, uint16_t filterSampleWindow, uint16_t filterThreshold);

void CLC_HAL_disablePWMClkCounting(void);
void CLC_HAL_enablePWMClkCounting(void);
void clc_Hal_setPWMinUpDownCountModeWithoutDeadBand(uint32_t base1, float32_t pwmFreq, float32_t pwmSysClkFreq);
void clc_Hal_setPwmUpDownCountModeWithDeadBand(uint32_t base1, float32_t pwmfreq, float32_t pwmSysClkFreq, float32_t red, float32_t fed);
void clc_Hal_setHRpwmUpDownCountModeWithDeadBand(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz, float32_t red_ns, float32_t fed_ns);
void clc_Hal_setPWMinUpDownCountMode(uint32_t base1, float32_t pwmFreq, float32_t pwmSysClkFreq);
void CLC_HAL_setupHRPWMinUpDownCount2ChAsymmetricMode(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz);
void CLC_HAL_setupECAPinPWMMode(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz);
void CLC_HAL_setupECAPinPWMMode_ISR3(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz);

void CLC_setSynOut(void);
void CLC_setSynIn(void);
void CLC_setSynIn2(void);

void CLC_HAL_setupCMPSS(uint32_t base1, float32_t current_limit, float32_t current_max_sense);
//
//CLA C Tasks defined in Cla1Tasks_C.cla
//
__attribute__((interrupt))  void Cla1Task1();
__attribute__((interrupt))  void Cla1Task2();
__attribute__((interrupt))  void Cla1Task3();
__attribute__((interrupt))  void Cla1Task4();
__attribute__((interrupt))  void Cla1Task5();
__attribute__((interrupt))  void Cla1Task6();
__attribute__((interrupt))  void Cla1Task7();
//__attribute__((interrupt))  void Cla1BackgroundTask();

extern uint16_t Cla1ProgLoadStart;
extern uint16_t Cla1ProgLoadEnd;
extern uint16_t Cla1ProgLoadSize;
extern uint16_t Cla1ProgRunStart;
extern uint16_t Cla1ProgRunEnd;
extern uint16_t Cla1ProgRunSize;

//
// ISR related
//
#if CLC_ISR1_RUNNING_ON == C28x_CORE

#ifndef __TMS320C28XX_CLA__
    #pragma CODE_SECTION(ISR1,"isrcodefuncs");
    #pragma INTERRUPT (ISR1, HPI)
    interrupt void ISR1(void);

    #pragma INTERRUPT (ISR1_secondTime, HPI)
    #pragma CODE_SECTION(ISR1_secondTime,"isrcodefuncs");
    interrupt void ISR1_secondTime(void);

    static inline void CLC_HAL_clearISR1InterruputFlag(void);
#endif

#endif

#if CLC_ISR2_RUNNING_ON == C28x_CORE

#ifndef __TMS320C28XX_CLA__
    #pragma CODE_SECTION(ISR2_primToSecPowerFlow,"isrcodefuncs");
    interrupt void ISR2_primToSecPowerFlow(void);
    static inline void CLC_HAL_clearISR2InterruputFlag(void);
#endif

#endif

#ifndef __TMS320C28XX_CLA__
    #pragma CODE_SECTION(ISR3,"ramfuncs");
    interrupt void ISR3(void);

    #pragma CODE_SECTION(sciaRxISR,"ramfuncs");
    interrupt void sciaRxISR(void);
    interrupt void sciaTxISR(void);
    interrupt void canaISR(void);
    interrupt void canbISR(void);
#endif

// Inline functions

extern int16_t tripIndicator;
static inline int16_t CLC_HAL_readTripFlags(void)
{
    if (XBAR_getInputFlagStatus(CLC_VPRIM_CMPSS_XBAR_FLAG1) || XBAR_getInputFlagStatus(CLC_VPRIM_CMPSS_XBAR_FLAG2))
    {
        tripIndicator = 1;
        XBAR_clearInputFlag(CLC_VPRIM_CMPSS_XBAR_FLAG1);
        XBAR_clearInputFlag(CLC_VPRIM_CMPSS_XBAR_FLAG2);
    }
    else if (XBAR_getInputFlagStatus(CLC_IPRIM_TANK_H_CMPSS_XBAR_FLAG) || XBAR_getInputFlagStatus(CLC_IPRIM_TANK_L_CMPSS_XBAR_FLAG))
    {
        tripIndicator = 2;
        XBAR_clearInputFlag(CLC_IPRIM_TANK_H_CMPSS_XBAR_FLAG);
        XBAR_clearInputFlag(CLC_IPRIM_TANK_L_CMPSS_XBAR_FLAG);
    }
    else if (XBAR_getInputFlagStatus(CLC_VSEC_CMPSS_XBAR_FLAG1) || XBAR_getInputFlagStatus(CLC_VSEC_CMPSS_XBAR_FLAG2))
    {
        tripIndicator = 3;
        XBAR_clearInputFlag(CLC_VSEC_CMPSS_XBAR_FLAG1);
        XBAR_clearInputFlag(CLC_VSEC_CMPSS_XBAR_FLAG2);
    }
    else if (XBAR_getInputFlagStatus(CLC_ISEC_CMPSS_XBAR_FLAG1) || XBAR_getInputFlagStatus(CLC_ISEC_CMPSS_XBAR_FLAG2))
    {
        tripIndicator = 4;
        XBAR_clearInputFlag(CLC_ISEC_CMPSS_XBAR_FLAG1);
        XBAR_clearInputFlag(CLC_ISEC_CMPSS_XBAR_FLAG2);
    }
    else
        tripIndicator = 0;

    return(tripIndicator);
}

static inline void CLC_HAL_resetSynchronousRectifierTripAction(uint16_t powerFlow)
{
    EALLOW;
    HWREGH(CLC_SEC_LEG1_PWM_BASE + EPWM_O_TZCTLDCA) = 0xFFFF;
    HWREGH(CLC_SEC_LEG1_PWM_BASE + EPWM_O_TZCTLDCB) = 0xFFFF;
    EDIS;
}

static inline void  CLC_HAL_setupSynchronousRectifierTripAction(uint16_t powerFlow)
{
    // no describe the behavior in case when DCAEVT2 and
    EPWM_setTripZoneAdvDigitalCompareActionA(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_LOW);
    EPWM_setTripZoneAdvDigitalCompareActionB(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_LOW);
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_updatePWMDutyPeriodPhaseShift)
static inline void CLC_HAL_updatePWMDutyPeriodPhaseShift(uint32_t period_ticks, uint32_t dutyAPrim_ticks, uint32_t dutyBPrim_ticks,
                                                         uint32_t dutyASec_ticks, uint32_t dutyBSec_ticks, uint32_t phaseShiftPrimSec_ticks)
{
    EALLOW;
    HWREG(CLC_PRIM_LEG1_PWM_BASE + HRPWM_O_TBPRDHR) = period_ticks;
    HWREG(CLC_PRIM_LEG1_PWM_BASE + HRPWM_O_CMPA) = dutyAPrim_ticks;
    HWREG(CLC_PRIM_LEG1_PWM_BASE + HRPWM_O_CMPB) = dutyBPrim_ticks;

    HWREG(CLC_SEC_LEG1_PWM_BASE + HRPWM_O_CMPA) = dutyASec_ticks;
    HWREG(CLC_SEC_LEG1_PWM_BASE + HRPWM_O_CMPB) = dutyBSec_ticks;
    HWREG(CLC_SEC_LEG1_PWM_BASE + EPWM_O_TBPHS) = phaseShiftPrimSec_ticks;

    EDIS;
}
#pragma FUNC_ALWAYS_INLINE(CLC_HAL_updatePWMDeadBandPrim)
static inline void CLC_HAL_updatePWMDeadBandPrim(uint32_t dbRED_ticks, uint32_t dbFED_ticks)
{
    EALLOW;
    HWREG(CLC_PRIM_LEG1_PWM_BASE + HRPWM_O_DBREDHR) = dbRED_ticks;
    HWREG(CLC_PRIM_LEG1_PWM_BASE + HRPWM_O_DBFEDHR) = dbFED_ticks;
    EDIS;
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setProfilingGPIO1)
static inline void CLC_HAL_setProfilingGPIO1(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING1_SET_REG) = CLC_GPIO_PROFILING1_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetProfilingGPIO1)
static inline void CLC_HAL_resetProfilingGPIO1(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING1_CLEAR_REG) = CLC_GPIO_PROFILING1_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

static inline void CLC_HAL_setProfilingGPIO2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  +  CLC_GPIO_PROFILING2_SET_REG ) = CLC_GPIO_PROFILING2_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

static inline void CLC_HAL_resetProfilingGPIO2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  +  CLC_GPIO_PROFILING2_CLEAR_REG ) = CLC_GPIO_PROFILING2_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

static inline void CLC_HAL_setProfilingGPIO3(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  +  CLC_GPIO_PROFILING3_SET_REG ) = CLC_GPIO_PROFILING3_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

static inline void CLC_HAL_resetProfilingGPIO3(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  +  CLC_GPIO_PROFILING3_CLEAR_REG ) = CLC_GPIO_PROFILING3_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setProfilingGPIO4)
static inline void CLC_HAL_setProfilingGPIO4(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING4_SET_REG) = CLC_GPIO_PROFILING4_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetProfilingGPIO4)
static inline void CLC_HAL_resetProfilingGPIO4(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING4_CLEAR_REG) = CLC_GPIO_PROFILING4_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setProfilingGPIO5)
static inline void CLC_HAL_setProfilingGPIO5(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING5_SET_REG) = CLC_GPIO_PROFILING5_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetProfilingGPIO5)
static inline void CLC_HAL_resetProfilingGPIO5(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING5_CLEAR_REG) = CLC_GPIO_PROFILING5_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setRELAY)
static inline void CLC_HAL_setRELAY(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_Relay_SET_REG) = CLC_GPIO_Relay_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetRELAY)
static inline void CLC_HAL_resetRELAY(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_Relay_CLEAR_REG) = CLC_GPIO_Relay_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}
/*
#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setProfilingGPIO7)
static inline void CLC_HAL_setProfilingGPIO7(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING7_SET_REG) = CLC_GPIO_PROFILING7_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetProfilingGPIO7)
static inline void CLC_HAL_resetProfilingGPIO7(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING7_CLEAR_REG) = CLC_GPIO_PROFILING7_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}
*/
/*
#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setProfilingGPIO8)
static inline void CLC_HAL_setProfilingGPIO8(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING8_SET_REG) = CLC_GPIO_PROFILING8_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetProfilingGPIO8)
static inline void CLC_HAL_resetProfilingGPIO8(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING8_CLEAR_REG) = CLC_GPIO_PROFILING8_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}
*/
/*
#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setProfilingGPIO9)
static inline void CLC_HAL_setProfilingGPIO9(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING9_SET_REG) = CLC_GPIO_PROFILING9_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_resetProfilingGPIO9)
static inline void CLC_HAL_resetProfilingGPIO9(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + CLC_GPIO_PROFILING9_CLEAR_REG) = CLC_GPIO_PROFILING9_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}
*/
#pragma FUNC_ALWAYS_INLINE(CLC_HAL_clearISR1PeripheralInterruptFlag)
static inline void CLC_HAL_clearISR1PeripheralInterruptFlag()
{
    EPWM_clearEventTriggerInterruptFlag(CLC_ISR1_PERIPHERAL_TRIG_BASE);
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_clearISR2PeripheralInterruptFlag)
static inline void CLC_HAL_clearISR2PeripheralInterruptFlag()
{
    ECAP_clearInterrupt(CLC_ISR2_ECAP_BASE, ECAP_ISR_SOURCE_COUNTER_PERIOD);
    ECAP_clearGlobalInterrupt(CLC_ISR2_ECAP_BASE);
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_clearISR3PeripheralInterruptFlag)
static inline void CLC_HAL_clearISR3PeripheralInterruptFlag()
{
    //ADC_clearInterruptStatus(CLC_ISR3_PERIPHERAL_TRIG_BASE, ADC_INT_NUMBER2);
    ECAP_clearInterrupt(CLC_ISR3_ECAP_BASE, ECAP_ISR_SOURCE_COUNTER_PERIOD);
    ECAP_clearGlobalInterrupt(CLC_ISR3_ECAP_BASE);
}

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_setupISR1Trigger)
static inline void CLC_HAL_setupISR1Trigger(float32_t freq)
{
    //
    // below sets up the ISR trigger for ISR1
    // ISR is triggered right before period value by a compare C match
    // as latency on C28x and CLA is different, for CLA a -20 is used
    // for C28x -27 is used based in GPIO toggle orbserved on the oscilloscope
    //
    #if CLC_ISR1_RUNNING_ON == CLA_CORE
        EPWM_setCounterCompareValue(CLC_ISR1_PERIPHERAL_TRIG_BASE,
                                       EPWM_COUNTER_COMPARE_C,
         ((TICKS_IN_PWM_FREQUENCY(freq, CLC_PWMSYSCLOCK_FREQ_HZ) >> 1) - 20));
    #else
        EPWM_setCounterCompareValue(CLC_ISR1_PERIPHERAL_TRIG_BASE, EPWM_COUNTER_COMPARE_C,
         ((TICKS_IN_PWM_FREQUENCY(freq, CLC_PWMSYSCLOCK_FREQ_HZ) >> 1) - 27));
    #endif

}

#ifndef __TMS320C28XX_CLA__
static inline void CLC_HAL_clearISR1InterruputFlag()
{
    Interrupt_clearACKGroup(CLC_ISR1_PIE_GROUP);
}

static inline void CLC_HAL_clearISR2InterruputFlag()
{
    Interrupt_clearACKGroup(CLC_ISR2_PIE_GROUP);
}

static inline void CLC_HAL_clearISR3InterruputFlag()
{
    Interrupt_clearACKGroup(CLC_ISR3_PIE_GROUP);
    ECAP_clearInterrupt(ECAP4_BASE, 0xFF);
    ECAP_clearGlobalInterrupt(ECAP4_BASE);
}

static inline void CLC_HAL_clearRxInterruputFlag()
{
    Interrupt_clearACKGroup(CLC_RxDY_PIE_GROUP);
}

static inline void CLC_HAL_setupInterrupt()
{

    EPWM_setInterruptSource(CLC_ISR1_PERIPHERAL_TRIG_BASE, EPWM_INT_TBCTR_U_CMPC);

    CLC_HAL_setupISR1Trigger(CLC_MIN_PWM_SWITCH_FREQ_HZ * 0.8);
    EPWM_setInterruptEventCount(CLC_ISR1_PERIPHERAL_TRIG_BASE, 1);
    EPWM_clearEventTriggerInterruptFlag(CLC_ISR1_PERIPHERAL_TRIG_BASE);
    EPWM_enableInterrupt(CLC_ISR1_PERIPHERAL_TRIG_BASE);

    ECAP_enableInterrupt(CLC_ISR2_ECAP_BASE, ECAP_ISR_SOURCE_COUNTER_PERIOD);
    //CPUTimer_enableInterrupt(CLC_TASKB_CPUTIMER_BASE);
    //CPUTimer_clearOverflowFlag(CLC_TASKB_CPUTIMER_BASE);
    //CPUTimer_enableInterrupt(CLC_ISR3_TIMEBASE);
    //CPUTimer_clearOverflowFlag(CLC_ISR3_TIMEBASE);

    ECAP_enableInterrupt(CLC_ISR3_ECAP_BASE, ECAP_ISR_SOURCE_COUNTER_PERIOD);

    /*    ADC_setInterruptSource(CLC_ISR3_PERIPHERAL_TRIG_BASE,
                           ADC_INT_NUMBER2, CLC_VPRIM_ADC_SOC_NO_13);  // org

    ADC_setInterruptSource(CLC_ISR3_PERIPHERAL_TRIG_BASE,                 // Herman
                           ADC_INT_NUMBER2, CLC_TH_HV_ADC_SOC_NO_14);

    ADC_enableInterrupt(CLC_ISR3_PERIPHERAL_TRIG_BASE, ADC_INT_NUMBER2);
    ADC_enableContinuousMode(CLC_ISR3_PERIPHERAL_TRIG_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(CLC_ISR3_PERIPHERAL_TRIG_BASE, ADC_INT_NUMBER2);  */

    //
    //Note the ISR1 is enabled in the PIE, though the peripheral interrupt is
    //not triggered until later
    //
    #if CLC_ISR1_RUNNING_ON == C28x_CORE
        Interrupt_register(CLC_ISR1_TRIG, &ISR1);
        CLC_HAL_clearISR1InterruputFlag();
        Interrupt_enable(CLC_ISR1_TRIG);
    #endif

    #if CLC_ISR2_RUNNING_ON == C28x_CORE
        Interrupt_register(CLC_ISR2_TRIG, &ISR2_primToSecPowerFlow);
        CLC_HAL_clearISR2InterruputFlag();
        Interrupt_enable(CLC_ISR2_TRIG);
    #endif

    Interrupt_register(CLC_ISR3_TRIG, &ISR3);
    CLC_HAL_clearISR3InterruputFlag();

    // Modify CLC_ISR3_TRIG
    Interrupt_enable(CLC_ISR3_TRIG);

    EALLOW;

    // Enable Global interrupt INTM
    EINT;

    // Enable Global real-time interrupt DBGM
    ERTM;
    EDIS;
}
#endif

#pragma FUNC_ALWAYS_INLINE(CLC_HAL_clearPWMTripFlags)
static inline void CLC_HAL_clearPWMTripFlags(uint32_t base)
{
    // clear all the configured trip sources for the PWM module
    EPWM_clearTripZoneFlag(base, (EPWM_TZ_INTERRUPT_OST | EPWM_TZ_INTERRUPT_CBC | EPWM_TZ_INTERRUPT_DCAEVT1));
}

static inline void CLC_HAL_clearPWMOneShotTripFlag(uint32_t base)
{
    // clear all the configured trip sources for the PWM module
    EPWM_clearTripZoneFlag(base, EPWM_TZ_INTERRUPT_OST);
}

static inline void CLC_HAL_forcePWMOneShotTrip(uint32_t base)
{
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
}
#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
