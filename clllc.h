// FILE:   clllc.h
//#############################################################################

#ifndef CLC_H
#define CLC_H

#ifdef __cplusplus

extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#ifndef __TMS320C28XX_CLA__
#include <math.h>
#else
#include <CLAmath.h>
#endif

#include "clllc_settings.h"
#include "clllc_hal.h"
#include "Parameter.h"
#include "Data.h"

//  self-define
#include "Ai_CLLLC_ADC_settings.h"
//

// Library header files
// DCL Library is used for the controller implementation
// GI -> DF13,
// GV -> DF13,

#ifndef __TMS320C28XX_CLA__
#include "DCLF32.h"
#define CLC_GI DCL_DF13
#define CLC_GV DCL_DF13

//
// GI on C28x ,run DCL_DF13 routines
//
#define CLC_GI_IMMEDIATE_RUN DCL_runDF13_C5
#define CLC_GI_PRECOMPUTE_RUN DCL_runDF13_C6

//
// GV on C28x ,run DCL_DF13 routines
//
#define CLC_GV_RUN DCL_runDF13_C4
#define CLC_GV_IMMEDIATE_RUN DCL_runDF13_C5
#define CLC_GV_PRECOMPUTE_RUN DCL_runDF13_C6

#else
#include "DCLCLA.h"
#define CLC_GI DCL_DF13_CLA
#define CLC_GV DCL_DF13_CLA

//
// GI on C28x ,run DCL_DF13 routines
//
#define CLC_GI_IMMEDIATE_RUN CLC_runDF13_L5
#define CLC_GI_PRECOMPUTE_RUN CLC_runDF13_L6

//
// GV on C28x ,run DCL_DF13 routines
//
#define CLC_GV_IMMEDIATE_RUN CLC_runDF13_L5
#define CLC_GV_PRECOMPUTE_RUN CLC_runDF13_L6

#pragma FUNC_ALWAYS_INLINE(CLC_runDF13_L5)
#pragma FUNC_ALWAYS_INLINE(CLC_runDF13_L6)

//
//! \brief          Executes an immediate 3rd order Direct Form 1 controller on
//!                 the FPU32, Implemented as inline C function
//! \param[in] p    Pointer to the DCL_DF13 controller structure
//! \param[in] ek   The servo error
//! \param[in] vk   The partial pre-computed control effort
//! \return         The control effort
//!
static inline float32_t CLC_runDF13_L5(DCL_DF13_CLA *p,
                                             float32_t ek,
                                             float32_t vk)
{
    p->d4 = (ek * p->b0) + vk;

    return(p->d4);
}

//
//! \brief          Executes a partial pre-computed 3rd order
//!                 Direct Form 1 controller on the FPU32
//!                 Implemented as inline C function
//!                 Note: d0 not used
//! \param[in] p    Pointer to the DCL_DF13 controller structure
//! \param[in] ek   The servo error
//! \param[in] uk   The controller output in the previous sample interval
//! \return         The control effort
//!
//
static inline float32_t CLC_runDF13_L6(DCL_DF13_CLA *p,
                                             float32_t ek,
                                             float32_t uk)
{
    float32_t v9;

    v9 = (ek * p->b1) + (p->d1 * p->b2) + (p->d2 * p->b3)
         - (uk * p->a1) - (p->d5 * p->a2) - (p->d6 * p->a3);
    p->d2 = p->d1;
    p->d1 = ek;
    p->d6 = p->d5;
    p->d5 = uk;

    return(v9);
}

#endif

#include "dlog_4ch.h"
#include "emavg.h"

#pragma FUNC_ALWAYS_INLINE(EPWM_setActionQualifierContSWForceAction)

#ifndef __TMS320C28XX_CLA__
#include "sfra_f32.h"
#include "sfra_gui_scicomms_driverlib.h"
#define CLC_SFRA_INJECT SFRA_F32_inject
#define CLC_SFRA_COLLECT SFRA_F32_collect
#else
#define CLC_SFRA_INJECT(m)    m
#define CLC_SFRA_COLLECT(m, n)
#endif

//
// Function to run the ISR3, for details see dataflow diagram in the user guide
//
void CLC_runISR3(void);
void CLC_Print(void);

void CLLC_Calculate_Rdc(void);
void CLLC_lookuptable(void);
//
// Function initialize the global variables
//
void clc_initGlobalVar(void);
//
// Function reads the trip flags and updates the board status enum type variable
//
void CLC_updateBoardStatus(void);

void CLC_runSFRABackGroundTasks(void);

void clc_BuildLvl_IndicaVar(void);
void CLC_chageSynchrosRectifiPwmBehavr(uint16_t powerFlow);

#ifndef __TMS320C28XX_CLA__
void CLC_setupSFRA();
#else
#endif

//
// typedefs
//

typedef union{
    enum
    {
        Lab1 = 0,
        Lab1_CLA = 1,
        Lab2 = 2,
        Lab2_CLA = 3,
        VoltCtrl = 4,
        VoltCtrl_CLA = 5,
        Lab4 = 6,
        Lab4_CLA = 7,
        Lab5 = 8,
        Lab5_CLA = 9,
        Lab6 = 10,
        Lab6_CLA = 11,
        Lab7 = 12,
        Lab7_CLA = 13,
        Lab8 = 14,
        Lab8_CLA = 15,
        undefinedLab = 12,
    }CLC_Lab_Enum;
    int32_t pad;
}CLC_Lab_EnumType;

extern  CLC_Lab_EnumType CLC_lab;

typedef union {
    enum {
        noTrip,
        prim_OverVolt,
        primTank_OverCurr,
        sec_OverVolt,
        sec_OverCurr,
    } CLC_TripFlag_Em;
    int32_t pad;
}CLC_TripFlag_EmType;

extern  CLC_TripFlag_EmType CLC_TripFlag;

typedef union {
    enum {
        pwmSwState_disabledAll = 0,
        SynchrosRectifi_OFF = 1,
        pwmSwSta_SynchrosRectifi_fixedDuty = 2,
        pwmSwSta_SynchrusRectifi_Act = 3
    }CLC_PwmSwState_Em;
    int32_t pad;
}CLC_PwmSwState_EmType;

extern  CLC_PwmSwState_EmType CLC_pwmSwStateAct, CLC_pwmSwState;

typedef union{
    enum
    {
        ac_dc_OFF = 0,
        ac_dc_PFC = 1,
        ac_dc_INV = 2,
    }CLC_CmdTo_AC_DC_Em;
    int32_t pad;
}CLC_CmdTo_AC_DC_EmType;

extern  CLC_CmdTo_AC_DC_EmType CLC_CmdTo_AC_DC;

extern CLC_GI CLC_gi;
extern float32_t CLC_giOut;
extern float32_t CLC_giError;
extern float32_t CLC_giPartialComputedValue;


extern CLC_GV CLC_gv;
extern float32_t CLC_gvOut;
extern float32_t CLC_gvError;
extern float32_t CLC_gvPartialComputedValue;

extern CLC_GV PSM_gv;
extern float32_t PSM_gvOut;
extern float32_t PSM_gvError;
extern float32_t PSM_gvPartialComputedValue;

extern CLC_GI PSM_gi;
extern float32_t PSM_giOut;
extern float32_t PSM_giError;
extern float32_t PSM_giPartialComputedValue;

//
// Flags for clearing trips and closing the loop
//
extern volatile int32_t CLC_closeGiLoop;
extern volatile int32_t CLC_closeGvLoop;
extern volatile int32_t CLC_clearTrip;

extern volatile float32_t CLC_pwmFrequencyRef_Hz;
extern volatile float32_t CLC_pwmFrequency_Hz;
extern volatile float32_t CLC_pwmFrequency_Hz_P1;
extern volatile float32_t CLC_pwmFrequencyPrev_Hz;

extern volatile float32_t CLC_pwmPeriodRef_pu;
extern float32_t CLC_pwmPeriod_pu;
extern float32_t CLC_pwmPeriodSlewed_pu;
extern float32_t CLC_pwmPeriodMin_pu;
extern float32_t CLC_pwmPeriodMax_pu;
extern float32_t CLC_pwmPeriodMax_ticks;
extern uint32_t CLC_pwmPeriod_ticks;

//
// 1- Primary Side (PFC-Inv/Bus)
//
extern float32_t CLC_iPrimSense_Amp;
extern float32_t CLC_iPrimSense_pu;
extern float32_t CLC_iPrimSenseOff_pu;
extern EMAVG CLC_iPrimSenseAvg_pu;

extern float32_t CLC_iPrimTankSense_Amp;
extern float32_t CLC_iPrimTankSense_pu;
extern float32_t CLC_iPrimTankSenseOff_pu;
extern float32_t CLC_iPrimTankSenseCalOff_pu;
extern float32_t CLC_iPrimTankSenseCalScale_pu;
extern EMAVG CLC_iPrimTankSenseAvg_pu;

extern float32_t CLC_vPrimSense_volt;
extern float32_t CLC_vPrimSense_pu;
extern float32_t CLC_vPrimSenseOff_pu;
extern float32_t CLC_vPrimSense_pu_ACC, CLC_vPrimSense_pu_flr;
extern EMAVG CLC_vPrimSenseAvg_pu;

extern float32_t CLC_vPrimRef_volt;
extern float32_t CLC_vPrimRef_pu;
extern float32_t CLC_vPrimRefSlewed_pu;

extern volatile float32_t CLC_pwmDutyPrimRef_pu;
extern float32_t CLC_pwmDutyPrim_pu;
extern uint32_t CLC_pwmDutyAPrim_ticks;
extern uint32_t CLC_pwmDutyBPrim_ticks;
extern float32_t CLC_SecDutyAng, CLC_SecDutyAngTick;

extern volatile float32_t CLC_pwmDeadBandREDPrimRef_ns;
extern uint32_t CLC_pwmDeadBandREDPrim_ticks;

extern volatile float32_t CLC_pwmDeadBandFEDPrimRef_ns;
extern uint32_t CLC_pwmDeadBandFEDPrim_ticks;

//
// 2-Secondary side (Battery)
//
extern float32_t CLC_iSecSense_Amp;
extern float32_t CLC_iSecSense_pu, CLC_iSecSense_pu_ACC, CLC_iSecSense_pu_flr;
extern float32_t CLC_iSecSenseOff_pu;
extern EMAVG CLC_iSecSenseAvg_pu;

extern volatile float32_t CLC_iSecRef_Amp;
extern float32_t CLC_iSecRef_pu;
extern float32_t CLC_iSecRefSlewed_pu;

extern float32_t CLC_vSecSense_volt;
extern float32_t CLC_vSecSenseCtrl_volt;
extern float32_t CLC_vSecSense_pu, CLC_vSecSense_pu_ACC, CLC_vSecSense_pu_flr;
extern float32_t CLC_vSecSenseCtrl_pu;
extern float32_t CLC_vSecSenseOff_pu;

extern float32_t CLC_vSecRef_volt;
extern float32_t CLC_vSecRef_pu;
extern float32_t CLC_vSecRefSlewed_pu;
extern EMAVG CLC_vSecSenseAvg_pu;

extern volatile float32_t CLC_pwmDutySecRef_pu;
extern float32_t CLC_pwmDutySec_pu;
extern uint32_t CLC_pwmDutyASec_ticks;
extern uint32_t CLC_pwmDutyBSec_ticks;

extern float32_t CLC_pwmDeadbandREDSec_ns;
extern uint16_t CLC_pwmDeadbandREDSec_ticks;

extern float32_t CLC_pwmDeadBandFEDSec_ns;
extern uint16_t CLC_pwmDeadbandFEDSec_ticks;

extern volatile float32_t CLC_pwmPhaseShiftPrimSecRef_ns;
extern float32_t CLC_pwmPhaseShiftPrimSec_ns;
extern int32_t CLC_pwmPhaseShiftPrimSec_ticks;
extern int16_t CLC_pwmPhaseShiftPrimSec_countDirection;

// Herman Temperature BUS port
extern float32_t CLC_THBUS_pu;
extern float32_t CLC_THBUSSense_Deg;
extern float32_t CLC_THbusSenseOff_pu;

// Herman Temperature HV port
extern float32_t CLC_TH_HV_pu;
extern float32_t CLC_TH_HV_Deg;
extern float32_t CLC_THhvSenseOff_pu;


extern volatile uint16_t CLC_pwmISRTrig_ticks;

extern volatile uint32_t CLC_cla_task_counter;

#ifndef __TMS320C28XX_CLA__

// datalogger
extern DLOG_4CH CLC_dLog1;
extern float32_t CLC_dBuff1[100], CLC_dBuff2[100], CLC_dBuff3[100], CLC_dBuff4[100];
extern float32_t CLC_dVal1, CLC_dVal2, CLC_dVal3, CLC_dVal4;
extern volatile float32_t CLC_dlogTrigger;

#if CLC_SFRA_TYPE != CLC_SFRA_DISABLED
extern float32_t CLC_plantMagVect[CLC_SFRA_FREQ_LENGTH];
extern float32_t CLC_plantPhaseVect[CLC_SFRA_FREQ_LENGTH];
extern float32_t CLC_olMagVect[CLC_SFRA_FREQ_LENGTH];
extern float32_t CLC_olPhaseVect[CLC_SFRA_FREQ_LENGTH];
extern float32_t CLC_freqVect[CLC_SFRA_FREQ_LENGTH];
#endif

extern SFRA_F32 CLC_sfra1;
extern int16_t SerialCommsTimer;
extern int16_t initializationFlag;
extern int16_t *varSetTxtList[];
extern int16_t *varSetBtnList[];
extern int16_t *varSetSldrList[];
extern int16_t *varGetList[];
extern int32_t *arrayGetList[];
extern int16_t *dataGetList[];
extern uint32_t *dataSetList[];

extern int16_t CommsOKflg, SerialCommsTimer;
#endif

// Herman Edit
extern float32_t flTHBUS_deg, flTHhv_deg, flBeta_Const;

//---- Herman Edit
extern float32_t flTH_BUS_R, flTH_HV_R, flTH_BUS_R0, flTH_HV_R0; // 10k+/-1%
extern float32_t flTHBUS_deg, flTHhv_deg, flBeta_Const;
extern float32_t flTemp0, flTHtmp;

extern bool system_OFF; // Input used to shut down the system

#define CLC_VSEC_MAX_SENSE_VOLTS_Start ((float32_t)520)

extern uint16_t dacVal;

extern uint16_t uwPSM_PhaseShift_ticks;
//extern float flPSM_PhaseShift_ticks_tmp;
extern float fl_PSM_shift_Angle, fl_PSM_shift_SlewedAngle;
extern uint16_t uwPSM_enable;
extern uint16_t uwPSM_Slave_enable, uwPSM_CurrCtrl_Master_enb;

extern uint16_t uwEnterInPSMstateCntLvl, uwEnterOutPSMstateCntLvl;

extern float flWindow_OFF_us;
extern float flWindow_LEN_us;

extern uint16_t CLC_ISEC_TANK_DACHVAL_reg, CLC_ISEC_TANK_DACLVAL_reg;
extern float32_t  CLC_SecTurnOff_us, CLC_SecTurnOffTick;
extern float32_t  CLC_SecTurnOn_us, CLC_SecTurnOnTick;
extern uint16_t uwCanTransmitCnt;
extern uint16_t uwCan_Transmit;

extern uint16_t DividFreq_Ctrl_cnt;
extern float Inv_CLC_VSEC_MAX_SENSE_VOLTS;

extern uint16_t receivedChar[64], writedChar[64];

#pragma FUNC_ALWAYS_INLINE(CLC_readSenseSignalsPrimToSecPowerFlow)
static inline void CLC_readSenseSignalsPrimToSecPowerFlow(void)
{
    CLC_iPrimSense_pu = ((float32_t)CLC_IPRIM_TANK_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_iPrimSenseOff_pu) * 2.0;

    CLC_iSecSense_pu = ((float32_t)CLC_ISEC_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_iSecSenseOff_pu) * 2.0;

    CLC_vPrimSense_pu = ((float32_t)CLC_VPRIM_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_vPrimSenseOff_pu);

    CLC_vSecSense_pu = ((float32_t)CLC_VSEC_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_vSecSenseOff_pu);
}

#pragma FUNC_ALWAYS_INLINE(CLC_readSenseSignalsSecToPrimPowerFlow)
static inline void CLC_readSenseSignalsSecToPrimPowerFlow(void)
{
    CLC_iPrimSense_pu = ((float32_t)CLC_IPRIM_TANK_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_iPrimSenseOff_pu) * -2.0;

    CLC_iSecSense_pu = ((float32_t)CLC_ISEC_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_iSecSenseOff_pu) * 2.0;

    CLC_vPrimSense_pu = ((float32_t)CLC_VPRIM_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR - CLC_vPrimSenseOff_pu);

    CLC_vSecSense_pu = ((float32_t)CLC_VSEC_OV_SAMPLE_ADC * CLC_ADC_PU_SCALE_FACTOR) - CLC_vSecSenseOff_pu;
}



#pragma FUNC_ALWAYS_INLINE(CLC_calculatePWMDutyPeriodPhaseShiftTicks_primToSecPowerFlow)
static inline void CLC_calculatePWMDutyPeriodPhaseShiftTicks_primToSecPowerFlow(void)
{
    uint32_t temp;

    // First calculate the hi-res ticks for the PWM
    // for this multiply by 2^16 , and divide by 1 (using left shift)
    // as the PWM is up down count mode
    temp = ((uint32_t)(((float32_t)(CLC_pwmPeriodSlewed_pu * CLC_pwmPeriodMax_ticks) *
                       (float32_t)TWO_RAISED_TO_THE_POWER_SIXTEEN)))>> 1;

    // next zero the lower 8 bits, as they are not part of TBPRDHR register
    CLC_pwmPeriod_ticks = temp & 0xFFFFFF00;

    // for hi-res the duty needs to set around period hence calculate
    // duty ticks as (period *(1-duty))
    CLC_pwmDutyAPrim_ticks = (uint32_t)((float32_t)CLC_pwmPeriod_ticks * (float32_t)(1 - fabsf(CLC_pwmDutyPrim_pu)));

    CLC_pwmDutyBPrim_ticks = CLC_pwmDutyAPrim_ticks;

    // the below is to get around the errata in HRPWM
    if((CLC_pwmDutyAPrim_ticks & 0x00FF00) == 0)
        CLC_pwmDutyAPrim_ticks = CLC_pwmDutyAPrim_ticks | 0x000100;

    if(CLC_SecTurnOn_us <= Min_CLC_SecTurnOn_us)
        CLC_SecTurnOn_us = Min_CLC_SecTurnOn_us;
    else if (CLC_SecTurnOn_us >= Max_CLC_SecTurnOn_us)
        CLC_SecTurnOn_us = Max_CLC_SecTurnOn_us;
    else{/* do nothing */}

    CLC_SecTurnOnTick = CLC_SecTurnOn_us*(float32_t)MicroSec_to_Tick;
    // not used it.

    CLC_pwmDutyASec_ticks = (uint32_t)(CLC_SecTurnOnTick);

    // 250kHz CLC_pwmPeriod_ticks is 4us : 13107200( HR PWM tick)
    CLC_pwmDutyBSec_ticks = (uint32_t)((float32_t)CLC_pwmPeriod_ticks - (CLC_SecTurnOnTick));


    CLC_pwmPhaseShiftPrimSec_ticks = ((int32_t)(CLC_pwmPeriod_ticks >> 1) - (int32_t)((float32_t)CLC_pwmPhaseShiftPrimSec_ns *
                                    CLC_PWMSYSCLOCK_FREQ_HZ * ONE_NANO_SEC * TWO_RAISED_TO_THE_POWER_SIXTEEN) + ((int32_t)2 << 16));

    CLC_pwmPhaseShiftPrimSec_ticks = CLC_pwmPhaseShiftPrimSec_ticks & 0xFFFF0000;

}

#pragma FUNC_ALWAYS_INLINE(CLC_calculatePWMDeadBandPrimTicks)
static inline void CLC_calculatePWMDeadBandPrimTicks(void)
{
    uint32_t ticks;
    // as we use double clock for the deadband there is a multiple by 2,
    // red_ticks= red_ns*pwm_clk_hz*one_ns*2^16
    // 2^16 multiply is (because of high res)
    ticks = ((uint32_t)(CLC_pwmDeadBandREDPrimRef_ns * (float32_t)TWO_RAISED_TO_THE_POWER_SIXTEEN * ((float32_t)ONE_NANO_SEC) * CLC_PWMSYSCLOCK_FREQ_HZ * 2.0f));
    CLC_pwmDeadBandREDPrim_ticks = (ticks & 0xFFFFFE00);

    ticks = ((uint32_t)(CLC_pwmDeadBandFEDPrimRef_ns * (float32_t)TWO_RAISED_TO_THE_POWER_SIXTEEN * ((float32_t)ONE_NANO_SEC) * CLC_PWMSYSCLOCK_FREQ_HZ * 2.0f));
    CLC_pwmDeadBandFEDPrim_ticks = (ticks & 0xFFFFFE00);

}
#pragma FUNC_ALWAYS_INLINE(EPWM_setCounterCompareValue)
#pragma FUNC_ALWAYS_INLINE(EPWM_enablePhaseShiftLoad)
#pragma FUNC_ALWAYS_INLINE(CLC_runISR1)
static inline void CLC_runISR1(void)
{
    CLC_HAL_updatePWMDutyPeriodPhaseShift(CLC_pwmPeriod_ticks, CLC_pwmDutyAPrim_ticks, CLC_pwmDutyBPrim_ticks,
                                          CLC_pwmDutyASec_ticks, CLC_pwmDutyBSec_ticks, CLC_pwmPhaseShiftPrimSec_ticks);

    EPWM_enablePhaseShiftLoad(CLC_SEC_LEG1_PWM_BASE);

    // write to COMPC or COMPD bits
    #pragma diag_suppress = 173
    HWREGH(CLC_ISR1_PERIPHERAL_TRIG_BASE + EPWM_O_CMPC) = CLC_pwmISRTrig_ticks;

    #pragma diag_warning = 173
    CLC_HAL_clearISR1PeripheralInterruptFlag();
}

#pragma FUNC_ALWAYS_INLINE(CLC_runISR1_secondTime)
static inline void CLC_runISR1_secondTime(void)
{
    EPWM_disablePhaseShiftLoad(CLC_SEC_LEG1_PWM_BASE);
    CLC_HAL_setupISR1Trigger(CLC_MIN_PWM_SWITCH_FREQ_HZ * 0.3);
    CLC_HAL_clearISR1PeripheralInterruptFlag();
}

#pragma FUNC_ALWAYS_INLINE(CLC_runISR2_primToSecPowerFlow)
//#pragma CODE_SECTION(CLC_runISR2_primToSecPowerFlow, "Cla1Prog");  Herman not used

static inline void CLC_runISR2_primToSecPowerFlow(void)
{
    uint16_t uwSyntmp1;
    float32_t flPSM_PhaseShift_ticks_tmp;

    // let curr ctrl form 100kHz to 1kHz
    CLC_readSenseSignalsPrimToSecPowerFlow();

    if(CLC_clearTrip == 1)
    {
        CLC_HAL_clearPWMTripFlags(CLC_PRIM_LEG1_PWM_BASE);
        CLC_HAL_clearPWMTripFlags(CLC_SEC_LEG1_PWM_BASE);

        asm(" RPT #1 || NOP");

        #if CLC_TEST_SETUP == CLC_TEST_SET_BATTERY
            CLC_closeGvLoop = 0;
            CLC_closeGiLoop = 1;
        #else
            CLC_closeGvLoop = 1;
            CLC_closeGiLoop = 0;
        #endif

        CLC_clearTrip = 0;

    }

    DividFreq_Ctrl_cnt++;
    CLC_vPrimSense_pu_ACC += CLC_vPrimSense_pu;
    CLC_iSecSense_pu_ACC += CLC_iSecSense_pu;
    CLC_vSecSense_pu_ACC += CLC_vSecSense_pu;

    if(DividFreq_Ctrl_cnt == (uint16_t)OSL_Curr_catch_time)
    {
        DividFreq_Ctrl_cnt = 0;
        CLC_iSecSense_pu_flr = CLC_iSecSense_pu_ACC*OSL_Curr_catch_time_ratio;
        CLC_vSecSense_pu_flr = CLC_vSecSense_pu_ACC*OSL_Curr_catch_time_ratio;
        CLC_vPrimSense_pu_flr = CLC_vPrimSense_pu_ACC*OSL_Curr_catch_time_ratio;
        CLC_iSecSense_pu_ACC = 0;
        CLC_vSecSense_pu_ACC = 0;
        CLC_vPrimSense_pu_ACC = 0;
    }

    CLC_HAL_clearISR2PeripheralInterruptFlag();

     /* Modify to change in the PWM on every time Interrupt */
    CLC_pwmDutyPrim_pu = CLC_pwmDutyPrimRef_pu;
    CLC_pwmDutySec_pu = CLC_pwmDutySecRef_pu;
    CLC_pwmPhaseShiftPrimSec_ns = CLC_pwmPhaseShiftPrimSecRef_ns;

    CLC_calculatePWMDutyPeriodPhaseShiftTicks_primToSecPowerFlow();

    CLC_HAL_setupISR1Trigger(CLC_pwmFrequencyPrev_Hz);

    CLC_pwmFrequencyPrev_Hz = CLC_pwmFrequency_Hz;

    #if CLC_ISR1_RUNNING_ON == CLA_CORE
          CLC_pwmISRTrig_ticks =
                 ((TICKS_IN_PWM_FREQUENCY(CLC_pwmFrequency_Hz,
                                      CLC_PWMSYSCLOCK_FREQ_HZ)>> 1) - 20);
    #else
          CLC_pwmISRTrig_ticks =
                 ((TICKS_IN_PWM_FREQUENCY(CLC_pwmFrequency_Hz,
                                      CLC_PWMSYSCLOCK_FREQ_HZ)>> 1) - 27);  //27
    #endif

    #if(PSM_Enable)
        flPSM_PhaseShift_ticks_tmp = (float)(CLC_pwmPeriod_ticks) * fl_PSM_shift_SlewedAngle;
        uwPSM_PhaseShift_ticks = (uint16_t)(((uint32_t)(flPSM_PhaseShift_ticks_tmp*0.0027777777777778f))>>15);  // 0.00277 = 1/360
    #endif

    #if(SynREC)
        uwSyntmp1 = (uint16_t)((flWindow_OFF_us)*95.58823529411765);  // fix 820ns turn on delay

        EPWM_setDigitalCompareWindowOffset(CLC_SEC_LEG1_PWM_BASE, uwSyntmp1);  // uwSyntmp1
        EPWM_setDigitalCompareWindowLength(CLC_SEC_LEG1_PWM_BASE, (uint16_t)((flWindow_LEN_us)*95.58823529411765));   // blank 1.15us
    #endif
}

#pragma FUNC_ALWAYS_INLINE(CLC_Ctrl)
static inline void CLC_Ctrl(void)
{
    static uint16_t uwEnterInPSMstateCnt = 0, uwEnterOutPSMstateCnt = 0;

    //CLC_iSecSense_pu_flr = CLC_iSecSense_pu_flr*Y_Filter_Curr + CLC_iSecSense_pu * X_Filter_Curr;

    if(CLC_closeGiLoop == 1)
    {
        #if Control_Type == TI_DCL
            CLC_giOut = CLC_GI_IMMEDIATE_RUN(&CLC_gi, CLC_giError, CLC_giPartialComputedValue);

        if(CLC_giOut > CLC_GI_OUT_MAX)
            CLC_giOut = CLC_GI_OUT_MAX;
        if(CLC_giOut < CLC_GI_OUT_MIN)
            CLC_giOut = CLC_GI_OUT_MIN;

        CLC_giPartialComputedValue = CLC_GI_PRECOMPUTE_RUN(&CLC_gi, CLC_giError, CLC_giOut);

        if(CLC_giOut < CLC_pwmPeriodMin_pu)
            CLC_giOut = CLC_pwmPeriodMin_pu;

        CLC_pwmPeriod_pu = CLC_giOut;

        #else
        CLC_giOut = CLLC_GV_antiWinup_PI(&CLLC_Gi_Antiwinup_PI, CLC_giError );

        CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /CLC_giOut);

        if(CLC_pwmPeriod_pu < CLC_pwmPeriodMin_pu)
            CLC_pwmPeriod_pu = CLC_pwmPeriodMin_pu;
        else if(CLC_pwmPeriod_pu > 1.0)
            CLC_pwmPeriod_pu = 1.0;

        #endif
    }
    else if(CLC_closeGvLoop == 1)
    {
        #if Control_Type == TI_DCL
            CLC_gvOut = CLC_GV_IMMEDIATE_RUN(&CLC_gv, CLC_gvError, CLC_gvPartialComputedValue);

            if(CLC_gvOut > CLC_GV_OUT_MAX)
                CLC_gvOut = CLC_GV_OUT_MAX;
            if(CLC_gvOut < CLC_GV_OUT_MIN)
                CLC_gvOut = CLC_GV_OUT_MIN;

            CLC_gvPartialComputedValue = CLC_GV_PRECOMPUTE_RUN(&CLC_gv, CLC_gvError, CLC_gvOut);

            if(CLC_gvOut < CLC_pwmPeriodMin_pu)
                CLC_gvOut = CLC_pwmPeriodMin_pu;

            CLC_pwmPeriod_pu = CLC_gvOut;

        #else
            if(uwPSM_enable == 0U)
            {
                CLC_gvOut = CLLC_GV_antiWinup_PI(&CLLC_Gv_Antiwinup_PI, CLC_gvError );

                if(CLC_gvOut == CLLC_Gv_Antiwinup_PI.Saturation_UpperSat)
                    uwEnterInPSMstateCnt++;
                else
                    uwEnterInPSMstateCnt = 0;

                CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /CLC_gvOut);

                if(CLC_pwmPeriod_pu < CLC_pwmPeriodMin_pu)
                {
                    CLC_pwmPeriod_pu = CLC_pwmPeriodMin_pu;
                }
                else if(CLC_pwmPeriod_pu > 1.0)
                {
                    CLC_pwmPeriod_pu = 1.0;
                }

                if(uwEnterInPSMstateCnt > uwEnterInPSMstateCntLvl)
                {
                    //For Master, It just turn PFM, not PSM
                    //uwPSM_enable = 1U;  // Chancel to input PSM mode
                    uwEnterInPSMstateCnt = 0;
                    uwEnterOutPSMstateCnt = 0;
                }else
                {
                     /* do nothing */
                }

                 //flOslPFMfreqCmd = CLC_gvOut;

            }
            else
            {
                PSM_gvOut = CLLC_GV_antiWinup_PI(&PSM_Gv_Antiwinup_PI, PSM_gvError );

                if(PSM_gvOut == PSM_MIN_angle)
                    uwEnterOutPSMstateCnt++;
                else
                    uwEnterOutPSMstateCnt = 0;

                if(uwEnterOutPSMstateCnt > uwEnterOutPSMstateCntLvl)
                {
                    uwPSM_enable = 0U;
                    uwEnterInPSMstateCnt = 0;
                    uwEnterOutPSMstateCnt = 0;
                }
                else
                {
                    /* do nothing */
                }

                fl_PSM_shift_Angle = PSM_gvOut;
            }
        #endif
    }
    else
    {
        /*
        CLC_gi.d4 = CLC_pwmPeriod_pu;
        CLC_gi.d5 = CLC_pwmPeriod_pu;
        CLC_gi.d6 = CLC_pwmPeriod_pu;
        CLC_gi.d7 = CLC_pwmPeriod_pu;

        CLC_gv.d4 = CLC_pwmPeriod_pu;
        CLC_gv.d5 = CLC_pwmPeriod_pu;
        CLC_gv.d6 = CLC_pwmPeriod_pu;
        CLC_gv.d7 = CLC_pwmPeriod_pu;

        CLC_giError = (CLC_iSecRefSlewed_pu - CLC_iSecSense_pu);
        CLC_gi.d0 = CLC_giError;
        CLC_gi.d1 = CLC_giError;
        CLC_gi.d2 = CLC_giError;
        CLC_gi.d3 = CLC_giError;

        CLC_giPartialComputedValue = CLC_pwmPeriod_pu;

        CLC_gvError = (CLC_vSecRefSlewed_pu - CLC_vSecSense_pu);
        CLC_gv.d0 = CLC_gvError;
        CLC_gv.d1 = CLC_gvError;
        CLC_gv.d2 = CLC_gvError;
        CLC_gv.d3 = CLC_gvError;

        CLC_gvPartialComputedValue = CLC_pwmPeriod_pu;
        */

        #if CLC_INCR_BUILD == CLC_OPEN_LOOP_BUILD
            #if CLC_SFRA_TYPE != CLC_SFRA_DISABLED
                CLC_pwmPeriod_pu = CLC_SFRA_INJECT(CLC_pwmPeriodRef_pu);
            #else
                CLC_pwmPeriod_pu = CLC_pwmPeriodRef_pu;
            #endif
        #else
            if(CLC_Initial_PFM_Hz> CLC_MAX_PWM_SWITCH_FREQ_HZ)
                CLC_Initial_PFM_Hz = CLC_MAX_PWM_SWITCH_FREQ_HZ;
            else if(CLC_Initial_PFM_Hz< CLC_MIN_PWM_SWITCH_FREQ_HZ)
                CLC_Initial_PFM_Hz = CLC_MIN_PWM_SWITCH_FREQ_HZ;
            CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /CLC_Initial_PFM_Hz);
        #endif
    }
}

#pragma FUNC_ALWAYS_INLINE(CLC_PSM_Ctrl)
static inline void CLC_PSM_Ctrl(void)
{
    static uint16_t uwEnterInPSMstateCnt = 0, uwEnterOutPSMstateCnt = 0;

    //CLC_iSecSense_pu_flr = CLC_iSecSense_pu_flr*Y_Filter_Curr + CLC_iSecSense_pu * X_Filter_Curr;
    //not use it.
    if(CLC_closeGiLoop == 1)
    {
        // CLC_iSecSense_pu_flr = CLC_iSecSense_pu_flr*Y_Filter_Curr + CLC_iSecSense_pu * X_Filter_Curr;
        /* Add CLC_iSecRefSlewed_pu limit by Vinput */
        CLC_giError = CLC_iSecSense_pu_flr - CLC_iSecRefSlewed_pu ;
        CLC_giOut = CLLC_GV_antiWinup_PI(&CLLC_Gi_Antiwinup_PI, CLC_giError );
        CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /CLC_giOut);
        if(CLC_pwmPeriod_pu < CLC_pwmPeriodMin_pu)
        {
        CLC_pwmPeriod_pu = CLC_pwmPeriodMin_pu;
        }
        else if(CLC_pwmPeriod_pu > 1.0)
        {
            CLC_pwmPeriod_pu = 1.0;
        }
    }
    else if(CLC_closeGvLoop == 1)
    {
        //CLC_gvError = ( CLC_vSecSense_pu - CLC_vSecRefSlewed_pu );  // CLC_vSecRefSlewed_pu - CLC_vSecSense_pu;
        PSM_gvError = ( CLC_vSecSense_pu - CLC_vSecRefSlewed_pu );

        if(uwPSM_enable == 0U)
        {
            CLC_gvOut = CLLC_GV_antiWinup_PI(&CLLC_Gv_Antiwinup_PI, CLC_gvError );

            if(CLC_gvOut == CLLC_Gv_Antiwinup_PI.Saturation_UpperSat)
                uwEnterInPSMstateCnt++;
            else
                uwEnterInPSMstateCnt = 0;

            CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /CLC_gvOut);

            if(CLC_pwmPeriod_pu < CLC_pwmPeriodMin_pu)
                CLC_pwmPeriod_pu = CLC_pwmPeriodMin_pu;
            else if(CLC_pwmPeriod_pu > 1.0)
                CLC_pwmPeriod_pu = 1.0;

            if(uwEnterInPSMstateCnt > uwEnterInPSMstateCntLvl)
            {
                //For Master, It just turn PFM, not PSM
                //uwPSM_enable = 1U;  // Chancel to input PSM mode
                uwEnterInPSMstateCnt = 0;
                uwEnterOutPSMstateCnt = 0;
            }
            else
            {
                 /* do nothing */
            }

            flOslPFMfreqCmd = CLC_gvOut;

        }
        else
        {
            PSM_gvOut = CLLC_GV_antiWinup_PI(&PSM_Gv_Antiwinup_PI, PSM_gvError);

            if(PSM_gvOut == PSM_MIN_angle)
                uwEnterOutPSMstateCnt++;
            else
                uwEnterOutPSMstateCnt = 0;

            if(uwEnterOutPSMstateCnt > uwEnterOutPSMstateCntLvl)
            {
                //uwPSM_enable = 0U;
                uwEnterInPSMstateCnt = 0;
                uwEnterOutPSMstateCnt = 0;
            }
            else
            {
                     /* do nothing */
            }

            fl_PSM_shift_Angle = PSM_gvOut;
        }

    }
    else
    {
        if(CLC_Initial_PFM_Hz> CLC_MAX_PWM_SWITCH_FREQ_HZ)
            CLC_Initial_PFM_Hz = CLC_MAX_PWM_SWITCH_FREQ_HZ;
        else if(CLC_Initial_PFM_Hz< CLC_MIN_PWM_SWITCH_FREQ_HZ)
            CLC_Initial_PFM_Hz = CLC_MIN_PWM_SWITCH_FREQ_HZ;

        CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /CLC_Initial_PFM_Hz);
        fl_PSM_shift_Angle = fl_PSM_shift_Angle_Init;
        fl_PSM_shift_SlewedAngle = fl_PSM_shift_Angle_Init;
    }
}

#pragma FUNC_ALWAYS_INLINE(CLLC_AvgCurr_Ctrl)
static inline void CLLC_AvgCurr_Ctrl(void)
{
    // CLC_iSecSense_pu_flr = CLC_iSecSense_pu_flr*Y_Filter_Curr + CLC_iSecSense_pu * X_Filter_Curr;
    PSM_giError = (CLC_iSecSense_pu_flr - CLC_iSecRef_pu );

    PSM_giOut = CLLC_GV_antiWinup_PI(&PSM_Gi_Antiwinup_PI, PSM_giError );
    fl_PSM_shift_Angle = PSM_giOut;
}

#pragma FUNC_ALWAYS_INLINE(CLC_StopPWM)
static inline void CLC_StopPWM(void)
{
    CLC_HAL_forcePWMOneShotTrip(CLC_PRIM_LEG1_PWM_BASE);
    CLC_HAL_forcePWMOneShotTrip(CLC_SEC_LEG1_PWM_BASE);
}

#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
