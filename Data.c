#include "stdint.h"
#include <stdbool.h>
// from OBC to BMS : format EA : Wake up Charge
// uint16_t uwRevBmsVolt, uwBMScurrOutRev;
uint16_t uwObcChargeStaSend;  // 0x11 is OBC is ready to charge.
uint16_t uwObcRecogBmSend;  //  0x11 is recognized BMS
uint16_t uwObcRecogGunSend;  //  0x11 is recognized Gun
uint16_t uwObcStart2BmsVoltStaSe;  //  0x11 is ready
uint16_t uwObc2BmsCommVerSe;  //  Communication Version Number

// from BMS to OBC : format EB : Wakeup Charge
//uint16_t uwRevBmsVolt, uwBMScurrOutRev;
uint16_t uwBmsSocRev; // Rated Capacity of the Battery
float    flBmsSocRev; // Rated Capacity of the Battery
uint16_t uwBmsChargTimes; // Charging times
uint16_t uwBms2ObcCommVer; // Communication Version Number

// from BMS to OBC : format C5 : Charge State
uint16_t uwBmsVsetMaxCmd, uwBmsIsetMaxCmd;
float    flBmsVsetMaxCmd, flBmsIsetMaxCmd;
uint16_t uwObcPwrOnMd,uwObcPwrOnMdCmd;
uint16_t uwObcCtrlMd, uwObcCtrlMdCmd;
uint16_t uwBmsRelayOnRev, uwBmsRelayOffRev;
uint16_t uwRevBmsVolt, uwRevBmsSoc, uwRevBmsChargTime, uwRevBmsCommVer;

float    flBmsVoltOutRev, flBmsCurrOutRev;

// from BMS to Brodcast : format C6 : Charge State
uint16_t uwBmsVoltRtn;
uint16_t uwBmsCurrRtn;
uint16_t uwBmsLowestTemperaRtn;
uint16_t uwBmsHighestTemperaRtn;
uint16_t uwBmsSocRtn;
uint16_t uwBmsChargRemainTimeRtn;
float flBmsVoltRtn;
float flBmsCurrRtn;
float flBmsLowestTemperaRtn;
float flBmsHighestTemperaRtn;
float flBmsSocRtn;

// from OBC to Broadcast : format C8 : Charge State
uint16_t uwObcVoltOutSend;
uint16_t uwObcCurrOutSend;
uint16_t uwObcStatusSend;
uint16_t uwObcErrorFlagSend;
uint16_t uwObcTemperatureSend;

// from OBC to Broadcast : format C0 : Stop Charge
uint16_t uwObcStopChargeActSe;
uint16_t uwObcDetSocFullWarnSe;
uint16_t uwObcDetBmsErrorSend;
uint16_t uwObcDetObcErrorSend;
//uint16_t uwObcStatusSend;
//uint16_t uwObcErrorFlagSend;
//uint16_t uwObcTemperatureSend;

// from BMS to Broadcast : format C1 : Stop Charge
uint16_t uwBmsApproachVoltRtn;
uint16_t uwBmsApproachSocRtn;
uint16_t uwBmsCmdObcStopCharg;  // 0x11 is stop
uint16_t uwBmsRelayOffStaRtn, uwBmsRelayOffCmd;
uint16_t uwBmsErrorFlag;

float flOslPFMfreqCmd, flOslVoltOutGUICmd;

// Cmd and Rtn is from BMS
// Send is from OBC
void voSetPwrOnMd(uint16_t uwPowerOnTmp);
void voSetCtrlMd(uint16_t uwPowerOnTmp);

enum State_GUI {UnitInit, SoftStart, OpenloopPFM_PSM, Const_Volt_Ctrl, Const_Curr_Ctrl} DcChargUnitSt, DcChargUnitSt_Cmd;
enum State_OnOff {On_Init, On, OFF} DcChargSt_On;

struct OBC_Status_st {
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

struct OBC_Status_st OBC_Status;

