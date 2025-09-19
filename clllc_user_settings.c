typedef struct CLLC_Herman_PI {
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
CLLC_PI CLLC_Gv_Antiwinup_PI, CLLC_Gi_Antiwinup_PI, PSM_Gv_Antiwinup_PI, PSM_Gi_Antiwinup_PI;

typedef struct PWMSTATE1 {
    unsigned short int PriPwmEnb_Act;
    unsigned short int PriPwmEnb_Sta;
    unsigned short int SecPwmEnb_Act;
    unsigned short int SecPwmEnb_Sta;
    unsigned short int SynRec_Act;
    unsigned short int SynRec_Sta;
}PWMSTATE;
PWMSTATE PWMSta;

unsigned short int  uwISEC_TANK_DACHVAL = 4095U;
unsigned short int  uwISEC_TANK_DACLVAL = 0U;

float CLC_Initial_PFM_Hz;
float CLC_GI_OUT_MAX_int, CLC_GI_OUT_MIN_int;

//float CLLC_GV_antiWinup_PI(CLLC_PI* CLLC_Antiwinup_PI ,float error );

float OSL_Curr_catch_time;
float OSL_Curr_catch_time_ratio;

float Inv_CLC_ISR3_FREQUENCY_HZ;

float CLLC_Master_Start_VoltInt;
float CLLC_Slave_Start_VoltInt;
float CLC_MAX_PERIOD_Slow_STEP_PUInt;
float CLC_MAX_PhaseAng_STEP_DegInt;
