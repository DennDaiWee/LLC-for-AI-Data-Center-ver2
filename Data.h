/*
 * Data.H
 *
 *  Created on: 2023¦~7¤ë10¤é
 *      Author: User
 */

#ifndef OSL_DATA_H_
#define OSL_DATA_H_

#include <stdint.h>
#include <stdbool.h>


// from OBC to BMS : format EA : Wake up Charge
// uint16_t uwRevBmsVolt, uwBMScurrOutRev;
extern uint16_t uwObcChargeStaSend;  // 0x11 is OBC is ready to charge.
extern uint16_t uwObcRecogBmSend;  //  0x11 is recognized BMS
extern uint16_t uwObcRecogGunSend;  //  0x11 is recognized Gun
extern uint16_t uwObcStart2BmsVoltStaSe;  //  0x11 is ready
extern uint16_t uwObc2BmsCommVerSe;  //  Communication Version Number

// from BMS to OBC : format EB : Wakeup Charge
//uint16_t uwRevBmsVolt, uwBMScurrOutRev;
extern uint16_t uwBmsSocRev; // Rated Capacity of the Battery
extern float    flBmsSocRev; // Rated Capacity of the Battery
extern uint16_t uwBmsChargTimes; // Charging times
extern uint16_t uwBms2ObcCommVer; // Communication Version Number

// from BMS to OBC : format C5 : Charge State
extern uint16_t uwBmsVsetMaxCmd, uwBmsIsetMaxCmd;
extern float    flBmsVsetMaxCmd, flBmsIsetMaxCmd;
extern uint16_t uwObcPwrOnMd,uwObcPwrOnMdCmd;
extern uint16_t uwObcCtrlMd, uwObcCtrlMdCmd;
extern uint16_t uwBmsRelayOnRev, uwBmsRelayOffRev;
extern uint16_t uwRevBmsVolt, uwRevBmsSoc, uwRevBmsChargTime, uwRevBmsCommVer;

extern float    flBmsVoltOutRev, flBmsCurrOutRev;

// from BMS to Brodcast : format C6 : Charge State
extern uint16_t uwBmsVoltRtn;
extern uint16_t uwBmsCurrRtn;
extern uint16_t uwBmsLowestTemperaRtn;
extern uint16_t uwBmsHighestTemperaRtn;
extern uint16_t uwBmsSocRtn;
extern uint16_t uwBmsChargRemainTimeRtn;
extern float flBmsVoltRtn;
extern float flBmsCurrRtn;
extern float flBmsLowestTemperaRtn;
extern float flBmsHighestTemperaRtn;
extern float flBmsSocRtn;

// from OBC to Broadcast : format C8 : Charge State
extern uint16_t uwObcVoltOutSend;
extern uint16_t uwObcCurrOutSend;
extern uint16_t uwObcStatusSend;
extern uint16_t uwObcErrorFlagSend;
extern uint16_t uwObcTemperatureSend;

// from OBC to Broadcast : format C0 : Stop Charge
extern uint16_t uwObcStopChargeActSe;
extern uint16_t uwObcDetSocFullWarnSe;
extern uint16_t uwObcDetBmsErrorSend;
extern uint16_t uwObcDetObcErrorSend;
//uint16_t uwObcStatusSend;
//uint16_t uwObcErrorFlagSend;
//uint16_t uwObcTemperatureSend;

// from BMS to Broadcast : format C1 : Stop Charge
extern uint16_t uwBmsApproachVoltRtn;
extern uint16_t uwBmsApproachSocRtn;
extern uint16_t uwBmsCmdObcStopCharg;  // 0x11 is stop
extern uint16_t uwBmsRelayOffStaRtn, uwBmsRelayOffCmd;
extern uint16_t uwBmsErrorFlag;


extern enum State_GUI {UnitInit, SoftStart, OpenloopPFM_PSM, Const_Volt_Ctrl, Const_Curr_Ctrl} DcChargUnitSt, DcChargUnitSt_Cmd;
extern enum State_OnOff {On_Init, On, OFF} DcChargSt_On;

extern float flOslPFMfreqCmd, flOslVoltOutGUICmd;

extern uint16_t ObcTemp,OBCACCurInf,OBCACVoltInf;

struct OBC_Status_st{
    uint16_t ObcTemp;
    uint16_t OBCACCurInf;
    uint16_t OBCACVoltInf;
    uint16_t CurrLim;
    bool PluginStatus;
    bool OBCEnabled;
    bool OBCOTP;
    bool OBCOOVP;
    bool OBCOUVP;
    bool OBCIOCP;
    bool OBCIOVP;
    bool OBCIUVP;
    bool OBCReady4Prechrg;
    bool OBCWakeup4Plugin;
    bool OBCFaultStatus;
    };
extern struct OBC_Status_st OBC_Status;

#endif /* OSL_DATA_H_ */
