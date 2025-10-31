//#############################################################################
//
// FILE:   clllc.c
//
// TITLE: This is the solution file.
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


//*****************************************************************************
// the includes
//*****************************************************************************

#include "clllc.h"


//
//--- System Related Globals ---
// Put the variables that are specific to control in the below section
// For example SFRA cannot run on CLA hence it must not be placed
// in the below section, the control verification using SFRA can only
// be carried out on the C28x
// Control Variables
//
// #pragma SET_DATA_SECTION("cla_shared")
//#pragma SET_DATA_SECTION("controlVariables")

CLC_Lab_EnumType CLC_Lab;

CLC_TripFlag_EmType CLC_tripFlag;

CLC_PwmSwState_EmType CLC_pwmSwStateAct, CLC_pwmSwState;

CLC_CmdTo_AC_DC_EmType CLC_CmdTo_AC_DC;

CLC_GI CLC_gi;
float32_t CLC_giOut;
float32_t CLC_giError;
float32_t CLC_giPartialComputedValue;

CLC_GI CLC_gv;
float32_t CLC_gvOut;
float32_t CLC_gvError;
float32_t CLC_gvPartialComputedValue;

CLC_GI PSM_gv;
float32_t PSM_gvOut;
float32_t PSM_gvError;
float32_t PSM_gvPartialComputedValue;

CLC_GI PSM_gi;
float32_t PSM_giOut;
float32_t PSM_giError;
float32_t PSM_giPartialComputedValue;

//
// Flags for clearing trips and closing the loop
//
volatile int32_t CLC_closeGiLoop;
volatile int32_t CLC_closeGvLoop;
volatile int32_t CLC_clearTrip;

volatile float32_t CLC_pwmFrequencyRef_Hz;
volatile float32_t CLC_pwmFrequency_Hz;
volatile float32_t CLC_pwmFrequencyPrev_Hz;

volatile float32_t CLC_pwmPeriodRef_pu;
float32_t CLC_pwmPeriod_pu;
float32_t CLC_pwmPeriodSlewed_pu;
float32_t CLC_pwmPeriodMin_pu;
float32_t CLC_pwmPeriodMax_pu;
float32_t CLC_pwmPeriodMax_ticks;
uint32_t CLC_pwmPeriod_ticks;

// 1- Primary Side (PFC-Inv/Bus)
uint16_t iPrimTank_ADCvalue, vPrim_ADCvalue, iSec_ADCvalue, vSec_ADCvalue;

float32_t CLC_iPrimTankSense_Amp, CLC_iPrimTankSense_pu, CLC_iPrimTankSenseOff_pu;
float32_t CLC_iPrimTankSenseCalOff_pu, CLC_iPrimTankSenseCalScale_pu;
EMAVG CLC_iPrimTankSenseAvg_pu;

float32_t CLC_vPrimSense_volt, CLC_vPrimSense_pu, CLC_vPrimSense_pu_ACC, CLC_vPrimSense_pu_flr;
float32_t CLC_vPrimSenseOff_pu;
EMAVG CLC_vPrimSenseAvg_pu;

float32_t CLC_vPrimRef_volt, CLC_vPrimRef_pu, CLC_vPrimRefSlewed_pu;

volatile float32_t CLC_pwmDutyPrimRef_pu;
float32_t CLC_pwmDutyPrim_pu;
uint32_t CLC_pwmDutyAPrim_ticks;
uint32_t CLC_pwmDutyBPrim_ticks;

volatile float32_t CLC_pwmDeadBandREDPrimRef_ns;
uint32_t CLC_pwmDeadBandREDPrim_ticks;

volatile float32_t CLC_pwmDeadBandFEDPrimRef_ns;
uint32_t CLC_pwmDeadBandFEDPrim_ticks;

// 2-Secondary side (Battery)
float32_t CLC_iSecSense_Amp, CLC_iSecSense_pu, CLC_iSecSense_pu_ACC, CLC_iSecSense_pu_flr;
float32_t CLC_iSecSenseOff_pu;
EMAVG CLC_iSecSenseAvg_pu;

volatile float32_t CLC_iSecRef_Amp;
float32_t CLC_iSecRef_pu, CLC_iSecRefSlewed_pu;

float32_t CLC_vSecSense_volt, CLC_vSecSenseCtrl_volt;
float32_t CLC_vSecSense_pu, CLC_vSecSense_pu_ACC, CLC_vSecSense_pu_flr, CLC_vSecSense_FB_Sim = 0;
float32_t CLC_vSecSenseCtrl_pu;
float32_t CLC_vSecSenseOff_pu;

float32_t CLC_vSecRef_volt;
float32_t CLC_vSecRef_pu;
float32_t CLC_vSecRefSlewed_pu;
EMAVG CLC_vSecSenseAvg_pu;

// Temperature BusTemp port
float32_t CLC_BusTemp_Sense_pu;
uint16_t CLC_BusTemp_Sense_Deg;
float32_t CLC_BusTemp_SenseOff_pu;
EMAVG CLC_BusTemp_Avg_pu;

// Temperature HV port
//float32_t CLC_THhv_pu;
//float32_t CLC_TH_hvSense_Deg;
//float32_t CLC_THhvSenseOff_pu;
//EMAVG CLC_THhvAvg_pu;

volatile float32_t CLC_pwmDutySecRef_pu;
float32_t CLC_pwmDutySec_pu;
uint32_t CLC_pwmDutyASec_ticks;
uint32_t CLC_pwmDutyBSec_ticks;

float32_t CLC_pwmDeadBandREDSec_ns;
uint16_t CLC_pwmDeadBandREDSec_ticks;

float32_t CLC_pwmDeadBandFEDSec_ns;
uint16_t CLC_pwmDeadbandFEDSec_ticks;

volatile float32_t CLC_pwmPhaseShiftPrimSecRef_ns;
float32_t CLC_pwmPhaseShiftPrimSec_ns;
int32_t CLC_pwmPhaseShiftPrimSec_ticks;


int16_t CLC_pwmPhaseShiftPrimSec_countDirection;

volatile uint16_t CLC_pwmISRTrig_ticks;

volatile uint32_t CLC_cla_task_counter;

#pragma SET_DATA_SECTION()

//#pragma DATA_SECTION(CLC_iSecSense_pu,"Cla1ToCpuMsgRAM");
//#pragma DATA_SECTION(CLC_iPrimSense_pu,"Cla1ToCpuMsgRAM");
//#pragma DATA_SECTION(CLC_vSecSense_pu,"Cla1ToCpuMsgRAM");
//#pragma DATA_SECTION(CLC_vPrimSense_pu,"Cla1ToCpuMsgRAM");

// datalogger
DLOG_4CH CLC_dLog1;
float32_t CLC_dBuff1[100], CLC_dBuff2[100], CLC_dBuff3[100], CLC_dBuff4[100];
float32_t CLC_dVal1, CLC_dVal2, CLC_dVal3, CLC_dVal4;
volatile float32_t CLC_dlogTrigger;

//--- SFRA Related Variables ---
SFRA_F32 CLC_sfra1;

//---- Herman Edit
uint16_t blSourceEna = 0;
float32_t flTH_BUS_R = 0, flTH_HV_R = 0, flTH_BUS_R0 = 10000, flTH_HV_R0 = 10000; // 10k+/-1%
float32_t flTHBUS_deg = 0, flTHhv_deg = 0, flBeta_Const = 3440;
float32_t flTemp0 = 25+273.15, flTHtmp;

bool system_OFF = false; // Input used to shut down the system

float Isec_realValue,Vsec_realValue, R_realValue;
float Isec_rValue_Fil = 0, Vsec_rValue_Fil = 0, Isec_rValue_Fil_pu = 0;

uint16_t uwPSM_PhaseShift_ticks;

float32_t fl_PSM_shift_Angle, fl_PSM_shift_SlewedAngle;

uint16_t uwPSM_enable = 0;
uint16_t uwPSM_Slave_enable = 0, uwPSM_CurrCtrl_Master_enb = 0;
uint16_t uwEnterInPSMstateCntLvl, uwEnterOutPSMstateCntLvl;

float flWindow_OFF_us = 0.0f;
float flWindow_LEN_us = 0.0f;

float32_t  CLC_SecTurnOff_us = 0, CLC_SecTurnOffTick = 0;
float32_t  CLC_SecTurnOn_us = 0, CLC_SecTurnOnTick = 0;

uint16_t CLC_ISEC_TANK_DACHVAL_reg = 4095, CLC_ISEC_TANK_DACLVAL_reg = 0;

uint16_t uwCanTransmitCnt = 0;
uint16_t uwCan_Transmit = 0;

uint16_t DividFreq_Ctrl_cnt = 0;


float Inv_CLC_VSEC_MAX_SENSE_VOLTS;

uint16_t TestStep1 = 0;
uint16_t TestStep2 = 0;
uint16_t TestStep3 = 0;
uint16_t TestStep4 = 0;
uint16_t TestStep5 = 0;
uint16_t TestStep6 = 0;
uint16_t TestStep7 = 0;
uint16_t TestStep8 = 0;
uint16_t TestStep9 = 0;

uint16_t ClearTestStep1 = 0;
uint16_t ClearTestStep2 = 0;
uint16_t ClearTestStep3 = 0;
uint16_t ClearTestStep4 = 0;
uint16_t ClearTestStep5 = 0;
uint16_t ClearTestStep6 = 0;
uint16_t ClearTestStep7 = 0;
uint16_t ClearTestStep8 = 0;
uint16_t ClearTestStep9 = 0;

uint16_t receivedChar[64], writedChar[64];

void CLC_Ctrl_slow(void);
void CLC_PSM_Ctrl_slow(void);

void CLC_runISR3(void)
{
    static uint16_t Avg_Curr_Ctrl = 0;
    static uint16_t CommErrCnt = 0;
    static uint16_t uwreadTripFlags;
    static uint16_t uwOn_times = 0, uwOFF_times = 0;

    CLC_vPrimSense_volt = CLC_vPrimSense_pu * CLC_VPRIM_MAX_SENSE_VOLTS;
    CLC_vSecSense_volt = CLC_vSecSense_pu * CLC_VSEC_MAX_SENSE_VOLTS;    // CLC_vSecSense_pu
    CLC_iPrimTankSense_Amp = CLC_iPrimTankSense_pu * CLC_IPRIM_TANK_MAX_SENSE_AMPS;
    CLC_iSecSense_Amp = CLC_iSecSense_pu * CLC_ISEC_MAX_SENSE_AMPS;
    CLC_BusTemp_Sense_Deg = Temp_cal(CLC_BusTemp_Sense_pu);
    //flOslVoltInFbk = CLC_vPrimSense_volt;
    uwObcVoltOutSend = (CLC_vSecSense_volt);

   switch(DcChargUnitSt)
   {
       case UnitInit:

           CLC_vSecRef_volt = (CLC_vPrimSense_volt * 1.0745341F - 25) * 0.8F;

           fl_PSM_shift_Angle = fl_PSM_shift_Angle_Zero;
           fl_PSM_shift_SlewedAngle = fl_PSM_shift_Angle_Zero;

           CLC_Initial_PFM_Hz = CLC_MAX_PWM_SWITCH_FREQ_HZ;
           flOslPFMfreqCmd = CLC_MAX_PWM_SWITCH_FREQ_HZ;

           /* Act Func */
           CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /flOslPFMfreqCmd);
           CLC_pwmPeriodSlewed_pu = CLC_pwmPeriod_pu;
           CLC_pwmFrequency_Hz = flOslPFMfreqCmd;

           CLLC_Gv_Antiwinup_PI.Integrator_DSTATE = CLC_MAX_PWM_SWITCH_FREQ_HZ;
           CLLC_Gi_Antiwinup_PI.Integrator_DSTATE = CLC_MAX_PWM_SWITCH_FREQ_HZ;
           PSM_Gv_Antiwinup_PI.Integrator_DSTATE = PSM_MIN_angle;
           PSM_Gi_Antiwinup_PI.Integrator_DSTATE = PSM_MIN_angle;

           Avg_Curr_Ctrl = 0;
           CommErrCnt = 0;
           uwPSM_enable = 0;
           CLC_closeGiLoop = 0;
           CLC_closeGvLoop = 0;

           PWMSta.PriPwmEnb_Act = 0U;
           PWMSta.SecPwmEnb_Act = 1U;  // prevent noise turn on

           //CLC_MAX_PhaseAng_STEP_DegInt = CLC_MAX_PhaseAng_STEP_Deg;
           CLC_MAX_PhaseAng_STEP_DegInt = 0.8;

           if(DcChargSt_On == On)
           {
               if(uwOn_times==0U)
                   CLC_clearTrip = 1U;
               uwOn_times++;

               if(uwOn_times > 3U)
               {
                   fl_PSM_shift_Angle = PSM_MAX_angle;
                   fl_PSM_shift_SlewedAngle = PSM_MAX_angle;
                   //PriPwmEnb_Act = 1U;
                   DcChargUnitSt = SoftStart;
                   uwOn_times = 0U;
               }
           }
           else
               DcChargUnitSt = UnitInit;
           break;

       case SoftStart: /*Real Soft start CLLC, Minimum out Volt */
           PWMSta.PriPwmEnb_Act = 1U;
           PWMSta.SecPwmEnb_Act = 1U;
           //re //flOslPFMfreqCmd = CLC_MAX_PWM_SWITCH_FREQ_HZ;

           /* Act Func */
           CLC_pwmPeriod_pu = (CLC_MIN_PWM_SWITCH_FREQ_HZ /flOslPFMfreqCmd);
           CLC_pwmPeriodSlewed_pu = CLC_pwmPeriod_pu;
           //CLC_pwmFrequency_Hz = (CLC_PWMSYSCLOCK_FREQ_HZ / (CLC_pwmPeriodSlewed_pu *
                                      //CLC_pwmPeriodMax_ticks));
           CLC_pwmFrequency_Hz = flOslPFMfreqCmd;

           //fl_PSM_shift_Angle = PSM_MAX_angle;
           //fl_PSM_shift_SlewedAngle = PSM_MAX_angle;

           uwreadTripFlags = CLC_HAL_readTripFlags();
           //re //if((CLC_vSecSense_volt > CLC_vPrimSense_volt * 0.5F) && (uwreadTripFlags == 0U) && (CLC_vSecSense_volt>120.0F)){
           CLC_closeGvLoop = 1;
           CLC_closeGiLoop = 0;

           uwPSM_enable = 0;

           CLC_vSecRef_volt = flOslVoltOutGUICmd;
           CLC_vSecRef_pu = CLC_vSecRef_volt * Inv_CLC_VSEC_MAX_SENSE_VOLTS;
           CLC_vSecRefSlewed_pu = (CLC_vSecRef_volt + CLC_vSecSense_volt)*0.5F* Inv_CLC_VSEC_MAX_SENSE_VOLTS;
           fl_PSM_shift_Angle = 0;

           if(fl_PSM_shift_SlewedAngle < (PSM_MIN_angle+0.01f))
           {
               PSM_Gv_Antiwinup_PI.blPWMEnb = 0U;
               CLLC_Gv_Antiwinup_PI.blPWMEnb = 1U;
               PSM_Gv_Antiwinup_PI.KiZ = 800.0F;
               CLLC_Gv_Antiwinup_PI.KiZ = 3500000.0F;
               CLLC_Gv_Antiwinup_PI.Saturation_LowerSat = CLC_MIN_PWM_SWITCH_FREQ_HZ;

               switch (DcChargUnitSt_Cmd)
               {
                   case Const_Volt_Ctrl:
                       DcChargUnitSt = Const_Volt_Ctrl;
                       break;
                   case Const_Curr_Ctrl:
                       DcChargUnitSt = Const_Volt_Ctrl;
                       break;
                   default : DcChargUnitSt = Const_Volt_Ctrl;
                       break;
               }

           }
           else
               DcChargUnitSt = SoftStart;
           //}
           break;

       case Const_Volt_Ctrl: /*Include volt Ctrl ref soft start */
           CLC_vSecRef_volt = flOslVoltOutGUICmd;

           CLC_Ctrl(); // Control volt Loop

           if((DcChargSt_On == OFF))
           {
               CLC_StopPWM();
               PWMSta.PriPwmEnb_Act = 0U;
               PWMSta.SecPwmEnb_Act = 1U;
               uwOFF_times++;

               if(uwOFF_times > 20U)
               {
                   DcChargUnitSt = UnitInit;
                   uwOFF_times = 0U;
               }
               else
                   DcChargUnitSt = Const_Volt_Ctrl;
           }
           else
               DcChargUnitSt = Const_Volt_Ctrl;
           break;

       case Const_Curr_Ctrl:
           break;
       case OpenloopPFM_PSM: /*Independent Mode */
           break;

       default:
           DcChargUnitSt = UnitInit;
   }

   CLC_HAL_clearISR3PeripheralInterruptFlag();
}

void clc_initGlobalVar(void)
{

    DCL_resetDF13(&CLC_gi);

    CLC_gi.a1 = CLC_GI1_2P2Z_A1;
    CLC_gi.a2 = CLC_GI1_2P2Z_A2;
    CLC_gi.a3 = CLC_GI1_2P2Z_A3;
    CLC_gi.b0 = CLC_GI1_2P2Z_B0;
    CLC_gi.b1 = CLC_GI1_2P2Z_B1;
    CLC_gi.b2 = CLC_GI1_2P2Z_B2;
    CLC_gi.b3 = CLC_GI1_2P2Z_B3;


    DCL_resetDF13(&CLC_gv);
    CLC_gv.a1 = CLC_GV1_2P2Z_A1;
    CLC_gv.a2 = CLC_GV1_2P2Z_A2;
    CLC_gv.a3 = CLC_GV1_2P2Z_A3;
    CLC_gv.b0 = CLC_GV1_2P2Z_B0;
    CLC_gv.b1 = CLC_GV1_2P2Z_B1;
    CLC_gv.b2 = CLC_GV1_2P2Z_B2;
    CLC_gv.b3 = CLC_GV1_2P2Z_B3;

    CLC_iPrimTankSense_Amp = 0;
    CLC_vPrimSense_volt = 0;
    CLC_iSecSense_Amp = 0;
    CLC_vSecSense_volt = 0;

    CLC_vSecSense_pu_ACC = 0;
    CLC_vSecSense_pu_flr = 0;
    CLC_iSecSense_pu_ACC = 0;
    CLC_iSecSense_pu_flr = 0;

    CLC_vPrimRef_volt = CLC_VPRIM_NOMINAL_VOLTS;
    CLC_vPrimRef_pu = CLC_VPRIM_NOMINAL_VOLTS / CLC_VPRIM_MAX_SENSE_VOLTS;

    CLC_vSecRef_volt = CLC_VSEC_NOMINAL_VOLTS;
    CLC_vSecRef_pu = CLC_VSEC_NOMINAL_VOLTS / CLC_VSEC_MAX_SENSE_VOLTS;
    CLC_vSecRefSlewed_pu = 0;

    CLC_pwmPeriod_pu = CLC_MIN_PWM_SWITCH_FREQ_HZ / CLC_NOMINAL_PWM_SWITCH_FREQ_HZ;
    CLC_pwmPeriodSlewed_pu = CLC_pwmPeriod_pu;

    CLC_pwmPeriodRef_pu = CLC_pwmPeriod_pu;
    CLC_pwmPeriodMin_pu = CLC_MIN_PWM_SWITCH_FREQ_HZ / CLC_MAX_PWM_SWITCH_FREQ_HZ;
    CLC_pwmPeriodMax_ticks = CLC_PWMSYSCLOCK_FREQ_HZ / CLC_MIN_PWM_SWITCH_FREQ_HZ;

    CLC_Initial_PFM_Hz = CLC_NOMINAL_PWM_SWITCH_FREQ_HZ;
    CLC_pwmFrequencyRef_Hz = CLC_NOMINAL_PWM_SWITCH_FREQ_HZ;
    CLC_pwmFrequency_Hz = CLC_NOMINAL_PWM_SWITCH_FREQ_HZ;
    CLC_pwmFrequencyPrev_Hz = CLC_pwmFrequency_Hz - 1.0;

    CLC_pwmPhaseShiftPrimSec_ns = CLC_PRIM_PWM_DEADBAND_RED_NS;  // org 81 : Herman Modify it for varied Deadtime
    CLC_pwmPhaseShiftPrimSecRef_ns = CLC_PRIM_PWM_DEADBAND_RED_NS; // org 81
    
    CLC_pwmDeadBandREDPrimRef_ns = CLC_PRIM_PWM_DEADBAND_RED_NS;
    CLC_pwmDeadBandFEDPrimRef_ns = CLC_PRIM_PWM_DEADBAND_FED_NS;

    CLC_vPrimSense_pu = 0;
    CLC_vPrimSenseOff_pu = 0;
    CLC_iSecSense_pu = 0;
    CLC_iSecSenseOff_pu = 0.5;
    CLC_vSecSense_pu = 0;
    CLC_vSecSenseOff_pu = 0;
    CLC_iPrimTankSense_pu = 0;
    CLC_iPrimTankSense_pu = 0;
    CLC_iPrimTankSenseOff_pu = 0.5;
    CLC_iPrimTankSenseOff_pu = 0.5;
//    CLC_THhvSenseOff_pu = 0;
    CLC_BusTemp_SenseOff_pu = 0;

    CLC_iPrimTankSenseCalOff_pu = 0;
    CLC_iPrimTankSenseCalScale_pu = 1;

    CLC_pwmDutyPrim_pu = 0.5;
    CLC_pwmDutyPrimRef_pu = 0.5;
    CLC_pwmDutySec_pu = 0.35;
    CLC_pwmDutySecRef_pu = 0.35;

    CLC_pwmSwState.CLC_PwmSwState_Em = SynchrosRectifi_OFF;
    CLC_pwmSwStateAct.CLC_PwmSwState_Em = SynchrosRectifi_OFF;

    CLC_CmdTo_AC_DC.CLC_CmdTo_AC_DC_Em = ac_dc_OFF;
    CLC_tripFlag.CLC_TripFlag_Em = noTrip;

    CLC_vPrimRef_volt = 600;
    CLC_closeGiLoop = 0;
    CLC_closeGvLoop = 0;
    CLC_clearTrip = 0;
    CLC_cla_task_counter = 0;

    // Herman
    CLLC_Gv_Antiwinup_PI.Init_PWM_freq = CLC_NOMINAL_PWM_SWITCH_FREQ_HZ;
    CLLC_Gv_Antiwinup_PI.Integrator = 0.0f;
    CLLC_Gv_Antiwinup_PI.Integrator_DSTATE = CLLC_Gv_Antiwinup_PI.Init_PWM_freq;
    CLLC_Gv_Antiwinup_PI.KiZ = 700000.0;  // fast response 7000000.0;
    CLLC_Gv_Antiwinup_PI.KpZ = 100000.0;  // fast response 1000000.0;
    CLLC_Gv_Antiwinup_PI.SampleT =  1/CLC_ISR3_FREQUENCY_HZ;
    CLLC_Gv_Antiwinup_PI.Saturation = 0.0f;

	CLLC_Gv_Antiwinup_PI.Saturation_LowerSat = CLC_MIN_PWM_SWITCH_FREQ_HZ ;
    CLLC_Gv_Antiwinup_PI.Saturation_UpperSat = CLC_MAX_PWM_SWITCH_FREQ_HZ;
    CLLC_Gv_Antiwinup_PI.UnitDelay_In = 0.0f;
    CLLC_Gv_Antiwinup_PI.UnitDelay_Out = 0.0f;
    CLLC_Gv_Antiwinup_PI.blPWMEnb = 0U;

    CLLC_Gi_Antiwinup_PI.Init_PWM_freq = CLC_NOMINAL_PWM_SWITCH_FREQ_HZ;
    CLLC_Gi_Antiwinup_PI.Integrator = 0.0f;
    CLLC_Gi_Antiwinup_PI.Integrator_DSTATE = CLLC_Gi_Antiwinup_PI.Init_PWM_freq;

	CLLC_Gi_Antiwinup_PI.KiZ = 700000.0;
    CLLC_Gi_Antiwinup_PI.KpZ = 600000.0;
    CLLC_Gi_Antiwinup_PI.SampleT =  1/CLC_ISR3_FREQUENCY_HZ;
    CLLC_Gi_Antiwinup_PI.Saturation = 0.0f;

    CLLC_Gi_Antiwinup_PI.Saturation_LowerSat = CLC_MIN_PWM_SWITCH_FREQ_HZ;
    CLLC_Gi_Antiwinup_PI.Saturation_UpperSat = CLC_MAX_PWM_SWITCH_FREQ_HZ;
    CLLC_Gi_Antiwinup_PI.UnitDelay_In = 0.0f;
    CLLC_Gi_Antiwinup_PI.UnitDelay_Out = 0.0f;
    CLLC_Gi_Antiwinup_PI.blPWMEnb = 0U;

// This is for PSM control
    PSM_Gv_Antiwinup_PI.Init_PWM_freq = fl_PSM_shift_Angle_Zero;  // be Init_PSM_angle
    PSM_Gv_Antiwinup_PI.Integrator = 0.0f;
    PSM_Gv_Antiwinup_PI.Integrator_DSTATE = PSM_Gv_Antiwinup_PI.Init_PWM_freq;
    PSM_Gv_Antiwinup_PI.KiZ = 5.0;
    PSM_Gv_Antiwinup_PI.KpZ = 0.0;
    PSM_Gv_Antiwinup_PI.SampleT = 1/CLC_ISR3_FREQUENCY_HZ;
    PSM_Gv_Antiwinup_PI.Saturation = 0.0f;
    PSM_Gv_Antiwinup_PI.Saturation_LowerSat = PSM_MIN_angle;
    PSM_Gv_Antiwinup_PI.Saturation_UpperSat = PSM_MAX_angle;
    PSM_Gv_Antiwinup_PI.UnitDelay_In = 0.0f;
    PSM_Gv_Antiwinup_PI.UnitDelay_Out = 0.0f;
    PSM_Gv_Antiwinup_PI.blPWMEnb = PSM_Gv_Antiwinup_PI_blPWMEnb;

    PSM_Gi_Antiwinup_PI.Init_PWM_freq = fl_PSM_shift_Angle_Zero;  // be Init_PSM_angle
    PSM_Gi_Antiwinup_PI.Integrator = 0.0f;
    PSM_Gi_Antiwinup_PI.Integrator_DSTATE = PSM_Gi_Antiwinup_PI.Init_PWM_freq;
    PSM_Gi_Antiwinup_PI.KiZ = 50.0;
    PSM_Gi_Antiwinup_PI.KpZ = 10.0;
    PSM_Gi_Antiwinup_PI.SampleT = 1/CLC_ISR3_FREQUENCY_HZ;
    PSM_Gi_Antiwinup_PI.Saturation = 0.0f;
    PSM_Gi_Antiwinup_PI.Saturation_LowerSat = PSM_MIN_angle;
    PSM_Gi_Antiwinup_PI.Saturation_UpperSat = PSM_MAX_angle;
    PSM_Gi_Antiwinup_PI.UnitDelay_In = 0.0f;
    PSM_Gi_Antiwinup_PI.UnitDelay_Out = 0.0f;
    PSM_Gi_Antiwinup_PI.blPWMEnb = PSM_Gi_Antiwinup_PI_blPWMEnb;

    PWMSta.PriPwmEnb_Act = 0U;
    PWMSta.PriPwmEnb_Sta = 0U;
    PWMSta.SecPwmEnb_Act = 0U;
    PWMSta.SecPwmEnb_Sta = 0U;
    PWMSta.SynRec_Act = 0U;
    PWMSta.SynRec_Sta = 0U;

    fl_PSM_shift_Angle = 0;
    fl_PSM_shift_SlewedAngle = 0;

    uwPSM_enable = 0;
    uwPSM_Slave_enable = 0;
    uwEnterInPSMstateCntLvl = 200;
    uwEnterOutPSMstateCntLvl = 20;

    /************************ SynRec Para. *****************************/
    flWindow_OFF_us = 0.0;  // 0.82us
    flWindow_LEN_us = 0.4;  // 1.0us   Minimum is 0.31us need check.
    CLC_ISEC_TANK_DACHVAL_reg = 2000U;  // For never be trig, use CLC_SecTurnOff_us to control turn on time
    CLC_ISEC_TANK_DACLVAL_reg = 2150U;  // It's real mode for least trig turn off.

    CLC_ISEC_TANK_DACHVAL_reg = 4095U;  // For must be low trig, use flWindow_LEN_us to control turn on time
    CLC_ISEC_TANK_DACLVAL_reg = 0U;     // not real mode

    uwISEC_TANK_DACHVAL = 2150U;  //set this is turn on level when SynRec_Act = 1
    uwISEC_TANK_DACLVAL = 1950U;  //set this is turn off level when SynRec_Act = 1

    CLC_SecTurnOff_us = 0;
    CLC_SecTurnOn_us = 0;

    CLC_SecTurnOffTick = 0;
    CLC_SecTurnOnTick = 0;
    /*********************************************************************/

    /* New State Mac. start */
    uwObcPwrOnMd = 0x0; // On
    uwObcPwrOnMdCmd = 0x0; // On

    uwObcCtrlMd = 0x2; // Voltage control
    uwObcCtrlMdCmd = 0x2; // Voltage control

    //flOslVoltOutCmd = CLC_VSEC_NOMINAL_VOLTS;

    Inv_CLC_VSEC_MAX_SENSE_VOLTS = 1.0/(float32_t)CLC_VSEC_MAX_SENSE_VOLTS;
    Inv_CLC_ISR3_FREQUENCY_HZ = 1.0/(float32_t)CLC_ISR3_FREQUENCY_HZ;

    OSL_Curr_catch_time = CLC_ISR2_FREQUENCY_HZ / CLC_TASKA_FREQ_HZ;
    OSL_Curr_catch_time_ratio = 1/OSL_Curr_catch_time;
    DividFreq_Ctrl_cnt = 0U;

    CLLC_Master_Start_VoltInt = CLLC_Master_Start_Volt;
    CLLC_Slave_Start_VoltInt = CLLC_Master_Start_Volt;

    CLC_MAX_PERIOD_Slow_STEP_PUInt = CLC_MAX_PERIOD_Slow_STEP_PU;
	CLC_MAX_PhaseAng_STEP_DegInt = CLC_MAX_PhaseAng_STEP_Deg;

	DcChargUnitSt = UnitInit;
	DcChargUnitSt_Cmd = UnitInit;

    AVG_reset(&CLC_vPrimSenseAvg_pu);
    AVG_config(&CLC_vPrimSenseAvg_pu, 0.5);
    AVG_reset(&CLC_vSecSenseAvg_pu);
    AVG_config(&CLC_vSecSenseAvg_pu, 0.5);
    AVG_reset(&CLC_iSecSenseAvg_pu);
    AVG_config(&CLC_iSecSenseAvg_pu, 0.01);  // org is 0.01
    AVG_reset(&CLC_iPrimTankSenseAvg_pu);
    AVG_config(&CLC_iPrimTankSenseAvg_pu, 0.0025);
    AVG_reset(&CLC_iPrimTankSenseAvg_pu);
    AVG_config(&CLC_iPrimTankSenseAvg_pu, 0.5);
    AVG_reset(&CLC_BusTemp_Avg_pu);
    AVG_config(&CLC_BusTemp_Avg_pu, 0.01);
//    AVG_reset(&CLC_THhvAvg_pu);
//    AVG_config(&CLC_THhvAvg_pu, 0.01);

    OBC_Status.ObcTemp = (100 + 48)/0.486;

    OBC_Status.PluginStatus = true;
    OBC_Status.OBCEnabled = true;
    OBC_Status.OBCOTP = false;
    OBC_Status.OBCOOVP = false;

    OBC_Status.OBCOUVP = false;
    OBC_Status.OBCIOCP = false;
    OBC_Status.OBCIOVP = false;
    OBC_Status.OBCIUVP = false;

    OBC_Status.OBCACCurInf = (30+50)/0.1;
    OBC_Status.OBCACVoltInf = (220)/0.1;

    OBC_Status.OBCReady4Prechrg = false;
    OBC_Status.OBCWakeup4Plugin = false;
    OBC_Status.OBCFaultStatus = false;

#if(0)
    DLOG_4CH_reset(&CLC_dLog1);
    DLOG_4CH_config(&CLC_dLog1,
                    &CLC_dVal1, &CLC_dVal2, &CLC_dVal3, &CLC_dVal4,
                    CLC_dBuff1, CLC_dBuff2, CLC_dBuff3, CLC_dBuff4,
                    100, 0.5, 1);  // 10Hz, Sample rate 500Hz, PreScale is 1 : delay one time
    CLC_dlogTrigger = 0;
#endif

}

void CLC_updateBoardStatus(void)
{
    int16_t tripStatusRead;
    tripStatusRead = CLC_HAL_readTripFlags();

    if(CLC_tripFlag.CLC_TripFlag_Em == noTrip)
    {
        switch (tripStatusRead)
        {
            case prim_OverVolt:
                CLC_tripFlag.CLC_TripFlag_Em = prim_OverVolt;
                break;
            case primTank_OverCurr:
                CLC_tripFlag.CLC_TripFlag_Em = primTank_OverCurr;
                break;
            case sec_OverVolt:
                CLC_tripFlag.CLC_TripFlag_Em = sec_OverVolt;
                break;
            case sec_OverCurr:
                CLC_tripFlag.CLC_TripFlag_Em = sec_OverCurr;
                break;
            default:
                break;
        }
    }
    else
    {
        if (tripStatusRead == 0)
            CLC_tripFlag.CLC_TripFlag_Em = noTrip;
        else
        {
            switch (tripStatusRead)
            {
                case prim_OverVolt:
                    CLC_tripFlag.CLC_TripFlag_Em = prim_OverVolt;
                    break;
                case primTank_OverCurr:
                    CLC_tripFlag.CLC_TripFlag_Em = primTank_OverCurr;
                    break;
                case sec_OverVolt:
                    CLC_tripFlag.CLC_TripFlag_Em = sec_OverVolt;
                    break;
                case sec_OverCurr:
                    CLC_tripFlag.CLC_TripFlag_Em = sec_OverCurr;
                    break;
                default:
                    break;
            }
        }
    }
}

void CLC_runSFRABackGroundTasks(void)
{
    SFRA_F32_runBackgroundTask(&CLC_sfra1);
    SFRA_GUI_runSerialHostComms(&CLC_sfra1);
}

void clc_BuildLvl_IndicaVar(void)
{
    #if CLC_LAB == 1
        #if CLC_CONTROL_RUNNING_ON == CLA_CORE
           CLC_Lab.CLC_Lab_Enum = Lab1_CLA;
        #else
           CLC_Lab.CLC_Lab_Enum = Lab1;
        #endif
    #elif CLC_LAB == 2
        #if CLC_CONTROL_RUNNING_ON == CLA_CORE
           CLC_Lab.CLC_Lab_Enum = Lab2_CLA;
        #else
           CLC_Lab.CLC_Lab_Enum = Lab2;
        #endif
    #elif CLC_LAB == 3
        #if CLC_CONTROL_RUNNING_ON == CLA_CORE
           CLC_Lab.CLC_Lab_Enum = VoltCtrl_CLA;
        #else
           CLC_Lab.CLC_Lab_Enum = VoltCtrl;
        #endif
    #elif CLC_LAB == 4
        #if CLC_CONTROL_RUNNING_ON == CLA_CORE
           CLC_Lab.CLC_Lab_Enum = Lab4_CLA;
        #else
           CLC_Lab.CLC_Lab_Enum = Lab4;
        #endif
    #elif CLC_LAB == 5
        #if CLC_CONTROL_RUNNING_ON == CLA_CORE
           CLC_Lab.CLC_Lab_Enum = Lab5_CLA;
        #else
            CLC_Lab.CLC_Lab_Enum = Lab5;
        #endif
   #else
           CLC_Lab.CLC_Lab_Enum = undefinedLab;
    #endif
}

void CLC_chageSynchrosRectifiPwmBehavr(uint16_t powerFlow)
{
    if(CLC_pwmSwState.CLC_PwmSwState_Em != CLC_pwmSwStateAct.CLC_PwmSwState_Em)
    {
        if(CLC_pwmSwState.CLC_PwmSwState_Em == pwmSwSta_SynchrosRectifi_fixedDuty)
        {
            CLC_HAL_resetSynchronousRectifierTripAction(powerFlow);
            CLC_pwmSwStateAct.CLC_PwmSwState_Em = pwmSwSta_SynchrosRectifi_fixedDuty;
            CLC_pwmPhaseShiftPrimSecRef_ns = CLC_PRIM_PWM_DEADBAND_RED_NS;
        }
        else if(CLC_pwmSwState.CLC_PwmSwState_Em == pwmSwSta_SynchrusRectifi_Act)
        {
            CLC_HAL_setupSynchronousRectifierTripAction(powerFlow);
            CLC_pwmSwStateAct.CLC_PwmSwState_Em = pwmSwSta_SynchrusRectifi_Act;
            CLC_pwmPhaseShiftPrimSecRef_ns = CLC_PRIM_PWM_DEADBAND_RED_NS;
        }
        else
            CLC_pwmSwStateAct.CLC_PwmSwState_Em = CLC_pwmSwState.CLC_PwmSwState_Em;

        clc_Hal_setPWMpins(CLC_pwmSwStateAct.CLC_PwmSwState_Em);
    }
}

void CLC_Print(void)
{
    static uint16_t rtb_Bias = 0, s25_iter = 0;
    static uint16_t TimeTag = 0;
    static uint16_t InxSciData[2] = {0,0};

    if (blSourceEna == 0x1)
    {
        rtb_Bias  = 11;
        s25_iter = 1;
        TimeTag++;
        do {
            switch (s25_iter)
            {
                case 1:
                    InxSciData[0] = 0x0A;
                    InxSciData[1] = 0x0B;
                    break;

                case 2:
                    InxSciData[0] = TimeTag & 0x00ff;
                    InxSciData[1] = (TimeTag & 0xff00)>>8 ;
                    break;

                case 3:
                    InxSciData[0] = (int16_t)(CLC_iPrimTankSense_Amp*1000) & 0x00ff;
                    InxSciData[1] = (((int16_t)(CLC_iPrimTankSense_Amp*1000)) & 0xff00) >>8;
                    break;

                case 4:
                    InxSciData[0] = (int16_t)(CLC_iSecSense_Amp*1000) & 0x00ff;
                    InxSciData[1] = (((int16_t)(CLC_iSecSense_Amp*1000)) & 0xff00)>>8;
                    break;

                case 5:
                    InxSciData[0] = (int16_t)(CLC_vPrimSense_volt*10) & 0x00ff;
                    InxSciData[1] = (((int16_t)(CLC_vPrimSense_volt*10)) & 0xff00)>>8;
                    break;

                case 6:
                    InxSciData[0] = (int16_t)(CLC_vSecSense_volt*10) & 0x00ff;
                    InxSciData[1] = (((int16_t)(CLC_vSecSense_volt*10)) & 0xff00)>>8;
                    break;

                case 7:
                    InxSciData[0] = (int16_t)(flTHBUS_deg*10) & 0x00ff;
                    InxSciData[1] = (((int16_t)(flTHBUS_deg*10)) & 0xff00)>>8;
                    break;

                case 8:
                    InxSciData[0] = (int16_t)(flTHhv_deg*10) & 0x00ff;
                    InxSciData[1] = (((int16_t)(flTHhv_deg*10)) & 0xff00)>>8;
                    break;

                case 9:
                    InxSciData[0] = (int16_t)(CLC_vSecSenseCtrl_volt*10) & 0x00ff;
                    InxSciData[1] = (((int16_t)(CLC_vSecSenseCtrl_volt*10)) & 0xff00)>>8;
                    //InxSciData[0] = 0;
                    //InxSciData[1] = 0;
                    break;

                case 10:
                    InxSciData[0] = 0x1D;  // Check Code
                    InxSciData[1] = 0x1C;  // Check Code
                    break;

                default:
                    InxSciData[0] = 0x1D;  // Temperature BUS
                    InxSciData[1] = 0x1C;  // Temperature BUS
                    break;
            }
            //scia_xmit((char*)&InxSciData, 2, 2);
            SCI_writeCharArray(SCIA_BASE,InxSciData,2);
            s25_iter++;
        } while ((rtb_Bias - s25_iter) != 0U);
    }
}

void CLLC_Calculate_Rdc(void){
}

void CLLC_lookuptable(void){
}

void CLC_PSM_Ctrl_slow(void)
{
    static uint16_t uwEnterInPSMstateCnt=0, uwEnterOutPSMstateCnt=0;

        if(CLC_closeGvLoop == 1)
         {
             PSM_gvError = (CLC_vSecSense_pu - CLC_vSecRefSlewed_pu);

             if(uwPSM_Slave_enable == 1U)
             {
                 PSM_gvOut = CLLC_GV_antiWinup_PI(&PSM_Gv_Antiwinup_PI, PSM_gvError);
                 if(PSM_gvOut == PSM_MIN_angle)
                     uwEnterOutPSMstateCnt++;
                 else
                     uwEnterOutPSMstateCnt = 0;

                 if(uwEnterOutPSMstateCnt > uwEnterOutPSMstateCntLvl)
                 {
                     //uwPSM_Slave_enable = 0U;
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
             fl_PSM_shift_Angle = fl_PSM_shift_Angle_Init;
             fl_PSM_shift_SlewedAngle = fl_PSM_shift_Angle_Init;
         }
}
