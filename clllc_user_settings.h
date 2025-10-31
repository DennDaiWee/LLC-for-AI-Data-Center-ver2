
// FILE:   clllc_user_settings.h
//#############################################################################

#ifndef PI_VALUE
#define PI_VALUE ((float32_t)3.141592653589)
#endif

#ifndef ONE_NANO_SEC
#define ONE_NANO_SEC ((float32_t)0.000000001)
#endif

#ifndef TWO_RAISED_TO_THE_POWER_SIXTEEN
#define TWO_RAISED_TO_THE_POWER_SIXTEEN ((float32_t)65536.0)
#endif

#ifndef TICKS_IN_PWM_FREQUENCY
#define TICKS_IN_PWM_FREQUENCY(m, n) (uint32_t)(n / (float32_t)m)
#endif

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

//  LLC pri. side HS and LS
#define CLC_PRIM_LEG1_PWM_BASE                   EPWM1_BASE
#define CLC_PRIM_LEG1_PWM_NO                     1
#define CLC_PRIM_LEG1_PWM_H_GPIO                 0
#define CLC_PRIM_LEG1_PWM_H_GPIO_PIN_CONFIG      GPIO_0_EPWM1A
#define CLC_PRIM_LEG1_PWM_H_DIS_GPIO_PIN_CONFIG  GPIO_0_GPIO0
#define CLC_PRIM_LEG1_PWM_L_GPIO                 1
#define CLC_PRIM_LEG1_PWM_L_GPIO_PIN_CONFIG      GPIO_1_EPWM1B
#define CLC_PRIM_LEG1_PWM_L_DIS_GPIO_PIN_CONFIG  GPIO_1_GPIO1

//  LLS sec. side HS and LS
#define CLC_SEC_LEG1_PWM_BASE                   EPWM2_BASE
#define CLC_SEC_LEG1_PWM_NO                     2
#define CLC_SEC_LEG1_PWM_H_GPIO                 2
#define CLC_SEC_LEG1_PWM_H_GPIO_PIN_CONFIG      GPIO_2_EPWM2A
#define CLC_SEC_LEG1_PWM_H_DIS_GPIO_PIN_CONFIG  GPIO_2_GPIO2
#define CLC_SEC_LEG1_PWM_L_GPIO                 3
#define CLC_SEC_LEG1_PWM_L_GPIO_PIN_CONFIG      GPIO_3_EPWM2B
#define CLC_SEC_LEG1_PWM_L_DIS_GPIO_PIN_CONFIG  GPIO_3_GPIO3

//  EPWM3A is to drive the cooling fan, EPWM3B will not be used
#define CLC_FAN_PWM_BASE                   EPWM3_BASE
#define CLC_FAN_PWM_NO                     3
#define CLC_FAN_PWM_H_GPIO                 4
#define CLC_FAN_PWM_H_GPIO_PIN_CONFIG      GPIO_4_EPWM3A
#define CLC_FAN_PWM_H_DIS_GPIO_PIN_CONFIG  GPIO_4_GPIO4
#define CLC_FAN_PWM_L_GPIO                 5
#define CLC_FAN_PWM_L_GPIO_PIN_CONFIG      GPIO_5_EPWM3B
#define CLC_FAN_PWM_L_DIS_GPIO_PIN_CONFIG  GPIO_5_GPIO5

//  EPWM4A/4B will not be used
#define CLC_SEC_LEG2_PWM_BASE                   EPWM4_BASE
#define CLC_SEC_LEG2_PWM_NO                     4
#define CLC_SEC_LEG2_PWM_H_GPIO                 6
#define CLC_SEC_LEG2_PWM_H_GPIO_PIN_CONFIG      GPIO_6_EPWM4A
#define CLC_SEC_LEG2_PWM_H_DIS_GPIO_PIN_CONFIG  GPIO_6_GPIO6
#define CLC_SEC_LEG2_PWM_L_GPIO                 7
#define CLC_SEC_LEG2_PWM_L_GPIO_PIN_CONFIG      GPIO_7_EPWM4B
#define CLC_SEC_LEG2_PWM_L_DIS_GPIO_PIN_CONFIG  GPIO_7_GPIO7

#define CLC_GLOBAL_LOAD_ENABLED FALSE

#define CLC_MAX_PERIOD_STEP_PU ((float32_t)0.05)
#define CLC_MAX_PERIOD_Slow_STEP_PU ((float32_t)0.05)
#define CLC_MAX_PhaseAng_STEP_Deg ((float32_t)0.35)

//ISR related
#define CLC_ISR1_PERIPHERAL_TRIG_BASE CLC_PRIM_LEG1_PWM_BASE
#define CLC_ISR1_TRIG INT_EPWM1
#define CLC_ISR1_PIE_GROUP INTERRUPT_ACK_GROUP3

#define CLC_ISR1_TRIG_CLA CLA_TRIGGER_EPWM1INT
#define CLC_ISR2_RUNNING_ON C28x_CORE

#define CLC_ISR2_ECAP_BASE ECAP1_BASE
#define CLC_ISR2_PWM_BASE  EPWM10_BASE  // from EPWM5_BASE to EPWM10_BASE
#define CLC_ISR2_TRIG INT_ECAP1
#define CLC_ISR2_PIE_GROUP INTERRUPT_ACK_GROUP4
#define CLC_ISR2_TRIG_CLA CLA_TRIGGER_ECAP1INT

#define CLC_ISR3_ECAP_BASE            ECAP4_BASE
#define CLC_ISR3_TRIG                 INT_ECAP4
#define CLC_ISR3_PIE_GROUP            INTERRUPT_ACK_GROUP4
#define CLC_ISR3_PERIPHERAL_TRIG_BASE ECAP4_BASE
#define CLC_ISR3_TRIG_CLA CLA_TRIGGER_ECAP4INT

#define CLC_RxDY_PIE_GROUP INTERRUPT_ACK_GROUP9
//
// Compensator related
//
#define CLC_GI_OUT_MAX   ((float32_t)0.98)
#define CLC_GI_OUT_MIN   ((float32_t)-0.1)

#define CLC_GV_OUT_MAX   ((float32_t)0.98)
#define CLC_GV_OUT_MIN   ((float32_t)-0.1)

#define CLC_VOLTS_PER_SECOND_SLEW ((float32_t)20.0)
#define CLC_AMPS_PER_SECOND_SLEW ((float32_t)1.0)


/*
#define CLC_THBUS_ADC_MODULE  ADCC_BASE
#define CLC_THBUS_ADC_PIN     ADC_CH_ADCIN1
#define CLC_THBUS_ADC_TRIG_SOURCE CLC_ADC_TRIG_SLOW_SOURCE
#define CLC_THBUS_ADC_ACQPS_SYS_CLKS 30
#define CLC_THBUS_ADCRESULTREGBASE ADCCRESULT_BASE

#define CLC_TH_HV_ADC_MODULE  ADCC_BASE
#define CLC_TH_HV_ADC_PIN     ADC_CH_ADCIN14
#define CLC_TH_HV_ADC_TRIG_SOURCE CLC_ADC_TRIG_SLOW_SOURCE
#define CLC_TH_HV_ADC_ACQPS_SYS_CLKS 30
#define CLC_TH_HV_ADCRESULTREGBASE ADCCRESULT_BASE
*/

#define CLC_TH_HV_ADCREAD (CLC_TH_HV_ADCREAD_14)


#if CLC_PROTECTION == CLC_PROTECT_ENB
// set 1 to enable the appropriate protection scheme
// CLC_BOARD_PROTECTION_ISEC and CLC_BOARD_PROTECTION_VPRIM 可以同時開
//#define CLC_BOARD_PROTECTION_IPRIM 0
#define CLC_BOARD_PROTECTION_IPRIM_TANK 0
#define CLC_BOARD_PROTECTION_ISEC 0 //1
#define CLC_BOARD_PROTECTION_VPRIM 0
#define CLC_BOARD_PROTECTION_VSEC 1//1
#define CLC_BOARD_PROTECTION_NTC 0
#define CLC_BOARD_PROTECTION_FANSPD 0
#else
#define CLC_BOARD_PROTECTION_IPRIM 0
#define CLC_BOARD_PROTECTION_ISEC 0
#define CLC_BOARD_PROTECTION_IPRIM_TANK 0
#define CLC_BOARD_PROTECTION_VPRIM 0
#define CLC_BOARD_PROTECTION_VSEC 0
#endif

#define CLC_IPRIM_TANK_CMPSS_XBAR_MUX          XBAR_MUX02
#define CLC_IPRIM_TANK_CMPSS_XBAR_MUX_VAL      XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L
#define CLC_IPRIM_TANK_CMPSS_XBAR_FLAG1  XBAR_INPUT_FLG_CMPSS2_CTRIPL
#define CLC_IPRIM_TANK_CMPSS_XBAR_FLAG2      XBAR_INPUT_FLG_CMPSS2_CTRIPH
/*
#define CLC_IPRIM_CMPSS_BASE CMPSS1_BASE
#define CLC_IPRIM_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_1  // CMPIN1P : ADCINA2
#define CLC_IPRIM_CMPSS_ASYSCTRL_CMPLPMUX  ASYSCTL_CMPLPMUX_SELECT_1  // CMPIN1N : ADCINA3
#define CLC_IPRIM_CMPSS_ASYSCTRL_MUX_VALUE 1  // change this from 3 to 1 on 221025

#define CLC_IPRIM_CMPSS_XBAR_MUX XBAR_MUX00
#define CLC_IPRIM_CMPSS_XBAR_MUX_VAL XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L
#define CLC_IPRIM_CMPSS_XBAR_FLAG1 XBAR_INPUT_FLG_CMPSS1_CTRIPH
#define CLC_IPRIM_CMPSS_XBAR_FLAG2 XBAR_INPUT_FLG_CMPSS1_CTRIPL
*/

//
#define CLC_VPRIM_CMPSS_BASE CMPSS1_BASE
#define CLC_VPRIM_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_1   // use C2 to detect
#define CLC_VPRIM_CMPSS_ASYSCTRL_CMPLPMUX  ASYSCTL_CMPLPMUX_SELECT_1   // use C2 to detect
#define CLC_VPRIM_CMPSS_ASYSCTRL_MUX_VALUE 1 // ref Technical reference manual : Table 9-2. Analog Pins and Internal Connections

#define CLC_VPRIM_CMPSS_XBAR_MUX XBAR_MUX00
#define CLC_VPRIM_CMPSS_XBAR_MUX_VAL XBAR_EPWM_MUX00_CMPSS1_CTRIPH
#define CLC_VPRIM_CMPSS_XBAR_FLAG1 XBAR_INPUT_FLG_CMPSS1_CTRIPH
#define CLC_VPRIM_CMPSS_XBAR_FLAG2 XBAR_INPUT_FLG_CMPSS1_CTRIPL
//
#define CLC_IPRIM_TANK_CMPSS_BASE CMPSS2_BASE
#define CLC_IPRIM_TANK_CMPSS_ASYSCTRL_CMPHPMUX ASYSCTL_CMPHPMUX_SELECT_2
#define CLC_IPRIM_TANK_CMPSS_ASYSCTRL_CMPLPMUX ASYSCTL_CMPLPMUX_SELECT_2
#define CLC_IPRIM_TANK_CMPSS_ASYSCTRL_MUX_VALUE 1

#define CLC_IPRIM_TANK_H_XBAR_MUX XBAR_MUX02
#define CLC_IPRIM_TANK_H_PWM_XBAR_MUX_VAL XBAR_EPWM_MUX02_CMPSS2_CTRIPH
#define CLC_IPRIM_TANK_H_OUT_XBAR_MUX_VAL XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH

#define CLC_IPRIM_TANK_L_XBAR_MUX XBAR_MUX03
#define CLC_IPRIM_TANK_L_PWM_XBAR_MUX_VAL XBAR_EPWM_MUX03_CMPSS2_CTRIPL
#define CLC_IPRIM_TANK_L_OUT_XBAR_MUX_VAL XBAR_OUT_MUX03_CMPSS2_CTRIPOUTL

#define CLC_IPRIM_TANK_H_CMPSS_XBAR_FLAG XBAR_INPUT_FLG_CMPSS2_CTRIPH
#define CLC_IPRIM_TANK_L_CMPSS_XBAR_FLAG XBAR_INPUT_FLG_CMPSS2_CTRIPL

#define CLC_IPRIM_TANK_DACHVAL 2150
#define CLC_IPRIM_TANK_DACLVAL 1950
//
#define CLC_VSEC_CMPSS_BASE CMPSS5_BASE
#define CLC_VSEC_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_5   // use C2 to detect
#define CLC_VSEC_CMPSS_ASYSCTRL_CMPLPMUX  ASYSCTL_CMPLPMUX_SELECT_5   // use C2 to detect
#define CLC_VSEC_CMPSS_ASYSCTRL_MUX_VALUE 1 // ref Technical reference manual : Table 9-2. Analog Pins and Internal Connections

#define CLC_VSEC_CMPSS_XBAR_MUX XBAR_MUX08
#define CLC_VSEC_CMPSS_XBAR_MUX_VAL XBAR_EPWM_MUX08_CMPSS5_CTRIPH_OR_L
#define CLC_VSEC_CMPSS_XBAR_FLAG1 XBAR_INPUT_FLG_CMPSS5_CTRIPH
#define CLC_VSEC_CMPSS_XBAR_FLAG2 XBAR_INPUT_FLG_CMPSS5_CTRIPL
//
#define CLC_ISEC_CMPSS_BASE CMPSS6_BASE
#define CLC_ISEC_CMPSS_ASYSCTRL_CMPHPMUX ASYSCTL_CMPHPMUX_SELECT_6  // Herman : org is ASYSCTL_CMPHPMUX_SELECT_6
#define CLC_ISEC_CMPSS_ASYSCTRL_CMPLPMUX ASYSCTL_CMPLPMUX_SELECT_6  // org is ASYSCTL_CMPLPMUX_SELECT_6
#define CLC_ISEC_CMPSS_ASYSCTRL_MUX_VALUE 1

#define CLC_ISEC_XBAR_MUX XBAR_MUX10
#define CLC_ISEC_XBAR_MUX_VAL XBAR_EPWM_MUX10_CMPSS6_CTRIPH_OR_L
#define CLC_ISEC_CMPSS_XBAR_FLAG1 XBAR_INPUT_FLG_CMPSS6_CTRIPH
#define CLC_ISEC_CMPSS_XBAR_FLAG2 XBAR_INPUT_FLG_CMPSS6_CTRIPL
//

// Set DAC H to 2150 and L to 1950
// This is done by emperical tuning of the board for efficiency
// IPRM TANK -> HSEC Pin 25 -> A6 (F28004x) -> CMPSS5
//

//-------------------------------------------------------------------------

// Debug related
// Active synchronous rectification debug
#define CLC_GPIO_XBAR1                      58
#define CLC_GPIO_XBAR1_PIN_CONFIG           GPIO_58_OUTPUTXBAR1

#define CLC_GPIO_XBAR2                      59
#define CLC_GPIO_XBAR2_PIN_CONFIG           GPIO_59_OUTPUTXBAR2

#define CLC_GPIO_XBAR4                      33
#define CLC_GPIO_XBAR4_PIN_CONFIG           GPIO_15_OUTPUTXBAR4

// GPIO profiling (DEBUG1)
#define CLC_GPIO_PROFILING1 36
#define CLC_GPIO_PROFILING1_SET_REG GPIO_O_GPBSET
#define CLC_GPIO_PROFILING1_CLEAR_REG GPIO_O_GPBCLEAR
#define CLC_GPIO_PROFILING1_SET GPIO_GPBSET_GPIO36
#define CLC_GPIO_PROFILING1_CLEAR GPIO_GPBCLEAR_GPIO36
#define CLC_GPIO_PROFILING1_PIN_CONFIG GPIO_36_GPIO36

// GPIO profiling (DEBUG2)
#define CLC_GPIO_PROFILING2 38
#define CLC_GPIO_PROFILING2_SET_REG GPIO_O_GPBSET
#define CLC_GPIO_PROFILING2_CLEAR_REG GPIO_O_GPBCLEAR
#define CLC_GPIO_PROFILING2_SET GPIO_GPBSET_GPIO38
#define CLC_GPIO_PROFILING2_CLEAR GPIO_GPBCLEAR_GPIO38
#define CLC_GPIO_PROFILING2_PIN_CONFIG GPIO_38_GPIO38

// GPIO profiling (DEBUG3)
#define CLC_GPIO_PROFILING3 61
#define CLC_GPIO_PROFILING3_SET_REG GPIO_O_GPASET
#define CLC_GPIO_PROFILING3_CLEAR_REG GPIO_O_GPACLEAR
#define CLC_GPIO_PROFILING3_SET GPIO_GPBSET_GPIO61
#define CLC_GPIO_PROFILING3_CLEAR GPIO_GPBCLEAR_GPIO61
#define CLC_GPIO_PROFILING3_PIN_CONFIG GPIO_61_GPIO61

// pri. PWM Enable
#define CLC_GPIO_PWMEnable 26
#define CLC_GPIO_PROFILING4_SET_REG GPIO_O_GPASET
#define CLC_GPIO_PROFILING4_CLEAR_REG GPIO_O_GPACLEAR
#define CLC_GPIO_PROFILING4_SET GPIO_GPASET_GPIO26
#define CLC_GPIO_PROFILING4_CLEAR GPIO_GPACLEAR_GPIO26
#define CLC_GPIO_PROFILING4_PIN_CONFIG GPIO_26_GPIO26
#define CLC_GPIO_PROFILING4_SET_REG GPIO_O_GPASET
#define CLC_GPIO_PROFILING4_RESET_REG GPIO_O_GPACLEAR

// sec. PWM Enable
#define CLC_GPIO_PWMEnable_SR 27
#define CLC_GPIO_PROFILING5_SET_REG GPIO_O_GPASET
#define CLC_GPIO_PROFILING5_CLEAR_REG GPIO_O_GPACLEAR
#define CLC_GPIO_PROFILING5_SET GPIO_GPASET_GPIO27
#define CLC_GPIO_PROFILING5_CLEAR GPIO_GPACLEAR_GPIO27
#define CLC_GPIO_PROFILING5_PIN_CONFIG GPIO_27_GPIO27
#define CLC_GPIO_PROFILING5_RESET_REG GPIO_O_GPACLEAR

// Only by test, not work for Relay
#define CLC_GPIO_Relay 20
#define CLC_GPIO_Relay_SET_REG GPIO_O_GPASET
#define CLC_GPIO_Relay_CLEAR_REG GPIO_O_GPACLEAR
#define CLC_GPIO_Relay_SET GPIO_GPASET_GPIO20
#define CLC_GPIO_Relay_CLEAR GPIO_GPACLEAR_GPIO20
#define CLC_GPIO_Relay_PIN_CONFIG GPIO_20_GPIO20
#define CLC_GPIO_Relay_RESET_REG GPIO_O_GPACLEAR


//#define CLC_GPIO_PROFILING7_SET_REG GPIO_O_GPASET
//#define CLC_GPIO_PROFILING7_CLEAR_REG GPIO_O_GPACLEAR
//#define CLC_GPIO_PROFILING7_SET GPIO_GPASET_GPIO9
//#define CLC_GPIO_PROFILING7_CLEAR GPIO_GPACLEAR_GPIO9
//#define CLC_GPIO_PROFILING7_PIN_CONFIG GPIO_9_GPIO9
//#define CLC_GPIO_PROFILING7_RESET_REG GPIO_O_GPACLEAR

//#define CLC_GPIO_PROFILING8_SET_REG GPIO_O_GPASET
//#define CLC_GPIO_PROFILING8_CLEAR_REG GPIO_O_GPACLEAR
//#define CLC_GPIO_PROFILING8_SET GPIO_GPASET_GPIO11
//#define CLC_GPIO_PROFILING8_CLEAR GPIO_GPACLEAR_GPIO11
//#define CLC_GPIO_PROFILING8_PIN_CONFIG GPIO_11_GPIO11
//#define CLC_GPIO_PROFILING8_RESET_REG GPIO_O_GPACLEAR

// Chk if is by test
//#define CLC_GPIO_PROFILING9_SET_REG GPIO_O_GPASET
//#define CLC_GPIO_PROFILING9_CLEAR_REG GPIO_O_GPACLEAR
//#define CLC_GPIO_PROFILING9_SET GPIO_GPASET_GPIO24
//#define CLC_GPIO_PROFILING9_CLEAR GPIO_GPACLEAR_GPIO24
//#define CLC_GPIO_PROFILING9_PIN_CONFIG GPIO_24_GPIO24
//#define CLC_GPIO_PROFILING9_RESET_REG GPIO_O_GPACLEAR

// LED blink
#define CLC_GPIO_LED1 34
#define CLC_GPIO_LED1_SET GPIO_GPASET_GPIO34
#define CLC_GPIO_LED1_CLEAR GPIO_GPACLEAR_GPIO34
#define CLC_GPIO_LED1_PIN_CONFIG GPIO_34_GPIO34

// LED blink (Blue)
#define CLC_GPIO_LED_BL 35
#define CLC_GPIO_LED_BL_SET GPIO_GPASET_GPIO35
#define CLC_GPIO_LED_BL_CLEAR GPIO_GPACLEAR_GPIO35
#define CLC_GPIO_LED_BL_PIN_CONFIG GPIO_35_GPIO35

// LED blink (Orange)
#define CLC_GPIO_LED_ORG 37
#define CLC_GPIO_LED_ORG_SET GPIO_GPASET_GPIO37
#define CLC_GPIO_LED_ORG_CLEAR GPIO_GPACLEAR_GPIO37
#define CLC_GPIO_LED_ORG_PIN_CONFIG GPIO_37_GPIO37

// ADC Related
#define CLC_ADC_PU_SCALE_FACTOR  ((float32_t)0.000244140625)
#define CLC_ADC_PU_PPB_SCALE_FACTOR ((float32_t)0.000488281250) //1/2^11

#define CLC_CMPSS_HYSTERESIS 2
#define CLC_CMPSSS_FILTER_PRESCALAR 2
#define CLC_CMPSS_WINODW 10
#define CLC_CMPSS_THRESHOLD 7

//CPU time related
#define CLC_TASKA_CPUTIMER_BASE CPUTIMER0_BASE
#define CLC_TASKB_CPUTIMER_BASE CPUTIMER1_BASE
#define CLC_TASKA_FREQ_HZ 50
#define CLC_TASKB_FREQ_HZ 50

#define CLC_GET_TASKA_TIMER_OVERFLOW_STATUS CPUTimer_getTimerOverflowStatus(CLC_TASKA_CPUTIMER_BASE)
#define CLC_CLEAR_TASKA_TIMER_OVERFLOW_FLAG CPUTimer_clearOverflowFlag(CLC_TASKA_CPUTIMER_BASE)

#define CLC_GET_TASKB_TIMER_OVERFLOW_STATUS CPUTimer_getTimerOverflowStatus(CLC_TASKB_CPUTIMER_BASE)
#define CLC_CLEAR_TASKB_TIMER_OVERFLOW_FLAG CPUTimer_clearOverflowFlag(CLC_TASKB_CPUTIMER_BASE)
/****************************************************************/
// Vin OV LV Protect
#define VinLVProt 0
#define CLC_Vin_LIMIT_volt 920
#define CLC_Vin_LowLIMIT_volt 500
#define SR_use_Absolute_curr 0

#define Control_Type 2
#define TI_DCL 1   // org control

#define Vout_MAX 220
#define Vout_Min 160

#define WC_Res            100.0 //1000.0       //Cutoff freq. 31 Hz of Vdc low pass filter
#define dT                (0.001/CLC_TASKC_FREQ_HZ) //(kHz)
#define Y_Filter_Res      (1.0/(1+WC_Res*dT))
#define X_Filter_Res      ((WC_Res*dT)/(1+WC_Res*dT))

#define ISR2_FREQUENCY  100.0 // kHz
#define WC_Curr         10.0 //1000.0       //Cutoff freq. 31 Hz of Curr low pass filter
#define dT2             (0.001/ISR2_FREQUENCY) //(kHz)
#define Y_Filter_Curr   (1.0/(1+WC_Curr*dT2))
#define X_Filter_Curr   ((WC_Curr*dT2)/(1+WC_Curr*dT2))

#define PSM_MAX_angle ((float32_t)35.0)
#define PSM_MIN_angle ((float32_t)0.0)  

#define PSM_Enable 1
#define PSM_Gv_Antiwinup_PI_blPWMEnb 0
#define PSM_Gi_Antiwinup_PI_blPWMEnb 0

#define SynREC  1

#define CanTest 0

#define CLLC_Master_Start_Volt 120.0F //400.0F
#define CLLC_Slave_Start_Volt CLLC_Master_Start_Volt

#define Min_CLC_SecTurnOff_us     0.415f
#define Min_CLC_SecTurnOn_us      1.15f

#define Max_CLC_SecTurnOn_us      2.5f
#define MicroSec_to_Tick          6553597.952000001f

#define CLC_SynCout XBAR_MUX14
#define CLC_SynCout_XBAR_MUX_VAL XBAR_OUT_MUX14_EXTSYNCOUT

#define ModBus 0

#define OpenloopStartCntLvl 10000U // 10sec Run at 4kHz
#define StartSlaveCntLvl    10000U // 10sec, Run at 10kHz

#define BeParrelCntLvl      10000U // 7.5sec, Run at 10kHz

#define unParreledCntLvl    10000U // 10sec Run at 4kHz
#define ParreledCntLvl      40000U // 10sec Run at 4kHz

#define CommErrCntLvl 7U

#define Avg_Curr_Ctrl_times (uint16_t)(CLC_TASKC_FREQ_HZ*0.001f)
#define Volt_Ctrl_times (uint16_t)10U

#define fl_PSM_shift_Angle_Init 9.0f
#define fl_PSM_shift_Angle_Zero 0.0f

#define Inv_CLC_MIN_PWM_SWITCH_FREQ_HZ ((float32_t)7.874015748031496e-6)

#define CLC_MIN_PWM_FREQUENCY_HZ_Parrel ((float32_t)180*1000)

typedef volatile struct CLLC_Herman_PI {
    unsigned short int blPWMEnb;
    float Init_PWM_freq;   //!< b0
    float Integrator;
    float Integrator_DSTATE;
    float KpZ;
    float KiZ;
    float Saturation_UpperSat;
    float Saturation_LowerSat;
    float Saturation;
    float UnitDelay_In;
    float UnitDelay_Out;
    float SampleT;
}CLLC_PI;
extern CLLC_PI CLLC_Gv_Antiwinup_PI, CLLC_Gi_Antiwinup_PI, PSM_Gv_Antiwinup_PI, PSM_Gi_Antiwinup_PI;

typedef struct PWMSTATE1 {
    unsigned short int PriPwmEnb_Act;
    unsigned short int PriPwmEnb_Sta;
    unsigned short int SecPwmEnb_Act;
    unsigned short int SecPwmEnb_Sta;
    unsigned short int SynRec_Act;
    unsigned short int SynRec_Sta;
}PWMSTATE;
extern PWMSTATE PWMSta;

extern float CLC_Initial_PFM_Hz;

extern float CLC_GI_OUT_MAX_int, CLC_GI_OUT_MIN_int;

extern uint16_t  uwISEC_TANK_DACHVAL;
extern uint16_t  uwISEC_TANK_DACLVAL;

extern float OSL_Curr_catch_time;
extern float OSL_Curr_catch_time_ratio;

extern float Inv_CLC_ISR3_FREQUENCY_HZ;

extern float CLLC_Master_Start_VoltInt;
extern float CLLC_Slave_Start_VoltInt;
extern float CLC_MAX_PERIOD_Slow_STEP_PUInt;
extern float CLC_MAX_PhaseAng_STEP_DegInt;

//static inline float CLLC_GV_antiWinup_PI(CLLC_PI* CLLC_Antiwinup_PI ,float error );

static inline float CLLC_GV_antiWinup_PI(CLLC_PI* CLLC_Antiwinup_PI, float error)
{
      float rtb_Sum1,rtb_Sum3;

      if (CLLC_Antiwinup_PI->blPWMEnb == 1U)
      {

        CLLC_Antiwinup_PI->Integrator = CLLC_Antiwinup_PI->Integrator_DSTATE;

        rtb_Sum3 = error * CLLC_Antiwinup_PI->KpZ + CLLC_Antiwinup_PI->Integrator;

        if (rtb_Sum3 > CLLC_Antiwinup_PI->Saturation_UpperSat)
            CLLC_Antiwinup_PI->Saturation = CLLC_Antiwinup_PI->Saturation_UpperSat;
        else if (rtb_Sum3 < CLLC_Antiwinup_PI->Saturation_LowerSat)
            CLLC_Antiwinup_PI->Saturation = CLLC_Antiwinup_PI->Saturation_LowerSat;
        else
            CLLC_Antiwinup_PI->Saturation = rtb_Sum3;

        if ((CLLC_Antiwinup_PI->UnitDelay_In == CLLC_Antiwinup_PI->UnitDelay_Out) ||
            (((CLLC_Antiwinup_PI->UnitDelay_In < CLLC_Antiwinup_PI->Saturation_LowerSat) ^ (error <= 0.0f))))
            rtb_Sum1 = CLLC_Antiwinup_PI->KiZ * CLLC_Antiwinup_PI->SampleT * error;
        else
            rtb_Sum1 = 0;

        /* Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' */
        CLLC_Antiwinup_PI->Integrator_DSTATE += rtb_Sum1;

        /* Update for UnitDelay: '<S14>/Unit Delay' */
        CLLC_Antiwinup_PI->UnitDelay_In = rtb_Sum3;
        CLLC_Antiwinup_PI->UnitDelay_Out = CLLC_Antiwinup_PI->Saturation;

        return CLLC_Antiwinup_PI->Saturation;

     }
     else
     {
        CLLC_Antiwinup_PI->Integrator_DSTATE = CLLC_Antiwinup_PI->Init_PWM_freq;
        CLLC_Antiwinup_PI->Saturation = CLLC_Antiwinup_PI->Integrator_DSTATE;

        return CLLC_Antiwinup_PI->Saturation;
     }
}

