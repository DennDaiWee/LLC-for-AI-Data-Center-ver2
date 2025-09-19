//#############################################################################
// TITLE:  CLLC is resonant DC DC converter
//#############################################################################

// 221004 Modify ADC allocation : add Ipri Isec Vpri Vsec VCtrl overSampling
// 221116 Add Modbus 03 06 cmd.
// 221122 Add Modbus Multi word
// 221129 finish Modbus CRC8,16 calc and lockup table
// 221130 enable Interruput : EPWM1 CMPC @Up(ISR1)(Sw. Freq), CLC_ISR2_ECAP_BASE (ISR2)(100kHz), ADCCINT2 (ISR3)(10kHz), SCI RX;
// 221130 use CPU timer2 TINT2 10kHz to read ADCC slow trig, and trig ADCCINT2(convert finish) to perform ISR3
// 230125 Merge Task_A0 to A1, Need to check Task_B Performane time
// 230607 Add CLLC 2sec turn on time : CLC_SecTurnOff_us ( before primary PWM turn off edge)
// ##############################################################################
// 230103 Need finish trip hold in N clock, If not satisfy condition, PWM output is low
// 230813 GPIO_setQualificationMode(CLC_GPIO_XBAR4, GPIO_QUAL_SYNC);
// 230825 2kHz control current PSM voltage
// 230916 Because Master need to do current share contrl, when Slave output volt is lower then Master.

// 230916 Then Master need output to lower voltage. so Master need to do current PSM control with voltage control of PFM control.
// 230916 But how to tune the Curr PSM gain(4kHz) and Volt PFM gain(1kHz)
// 230916 It need to tune Curr by fast Freq. Hope to tune Volt one time  and then tune Curr 4 times.
// 230916 But Volt gain need be high, and Curr gain need be lower then Slave curr gain.
// ##############################################################################
// 230929 "this Problem ?" (search item) : ISR1 Trig used Previous PwmFrequency, But ISR2 PWM period use this new PwmPrequency
// 231014 Test_Add_Slave2_Curr_Avg_by_2_CLLC
// Mod ISR3 : Paralled Mode : flOslAvgCurrOutCmd = (flOslCurrOutFbk_Slave1 + flOslCurrOutFbk_Slave2 + CLC_iSecSense_Amp)*0.333333333333f;
// to  flOslAvgCurrOutCmd = (flOslCurrOutFbk_Slave1 + CLC_iSecSense_Amp)*0.5f;

// 231030 Mod CLC_Ctrl_slow() on Normal CLLC1-CLLC2 communication

// 231218 use GP Timer2 to do GUI Interface
// 可用的IO : CLC_HAL_setProfilingGPIO2,3,6
// 250618 刪除 CAN 同步機制，保留編寫成 CAN_FD 同步的架構

#include "clllc.h"
#include "Parameter.h"

void (*Alpha_State_Ptr)(void);  // Base States pointer
void Task_A0(void);  //state Task_A0
void (*A_Task_Ptr)(void);       // State pointer A branch
void Task_A1(void);  //state Task_A1

//####################################################################
// Note that the watchdog is disabled in codestartbranch.asm
// for this project. This is to prevent it from expiring while
// c_init routine initializes the global variables before reaching the main()
//############   Herman Add : General Function  ########################################################
void DAC_init(void);
void clc_ADC_Offset_Calc(void);
void HAL_init();
//**************************** DAC use variable ***************************
uint16_t DAC_switch = 0, DAC_Val = 2048;
//***********************************************************************

bool send_En = false;

// ISR3 and ISR2*10time = ISR3, but maybe delay one cycle.
void main(void)
{
    // initializing PLL, copying code from FLASH to RAM,
    // initialize the CPU timers in the background B task

    HAL_init();
	EINT;

    while(1)
    {
        // A0-A1
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,A1,...)

        if(DAC_switch == 1)
        {
            DAC_setShadowValue(DACA_BASE, DAC_Val);   // Feedback Volt
            DEVICE_DELAY_US(2);
            DAC_setShadowValue(DACB_BASE, (CLC_gvOut/CLC_MAX_PWM_SWITCH_FREQ_HZ*4095U));
            DEVICE_DELAY_US(2);
        }

        if(PWMSta.PriPwmEnb_Act == 1U & PWMSta.PriPwmEnb_Sta == 0U)
        {
            CLC_HAL_setProfilingGPIO4();
            PWMSta.PriPwmEnb_Sta = 1U;
            //CLC_clearTrip = 1;
        }
        else if(PWMSta.PriPwmEnb_Act == 0U & PWMSta.PriPwmEnb_Sta == 1U)
        {
            CLC_HAL_resetProfilingGPIO4();
            PWMSta.PriPwmEnb_Sta = 0U;
            //CLC_StopPWM();
        }
        else
        {
            /* nothing */
        }

        if(PWMSta.SecPwmEnb_Act == 1U & PWMSta.SecPwmEnb_Sta == 0U)
        {
            CLC_HAL_setProfilingGPIO5();
            PWMSta.SecPwmEnb_Sta = 1U;
        }
        else if(PWMSta.SecPwmEnb_Act == 0U & PWMSta.SecPwmEnb_Sta == 1U)
        {
            CLC_HAL_resetProfilingGPIO5();
            PWMSta.SecPwmEnb_Sta = 0U;
        }
        else
        {
            /* nothing */
        }
    }
}

#if CLC_ISR1_RUNNING_ON == C28x_CORE
interrupt void ISR1(void)
{
//    CLC_HAL_setProfilingGPIO6();

    CLC_runISR1();
    CLC_HAL_clearISR1InterruputFlag();

//    CLC_HAL_resetProfilingGPIO6();

    Interrupt_register(CLC_ISR1_TRIG, &ISR1_second);
}
#endif

#if CLC_ISR1_RUNNING_ON == C28x_CORE
interrupt void ISR1_second(void)
{
  //
  //CLC_HAL_setProfilingGPIO2();
  //
    CLC_runISR1_secondTime();
    CLC_HAL_clearISR1InterruputFlag();

  //CLC_HAL_resetProfilingGPIO2();

    Interrupt_register(CLC_ISR1_TRIG, &ISR1);
}
#endif

#if CLC_ISR2_RUNNING_ON == C28x_CORE
interrupt void ISR2_primToSecPowerFlow(void)
{
    //
    // enable group 3 interrupt only to interrupt ISR2
    //
    IER |= 0x4;
    IER &= 0x4;
    EINT;
    CLC_HAL_setProfilingGPIO2();
    CLC_runISR2_primToSecPowerFlow();
    CLC_HAL_resetProfilingGPIO2();
    DINT;
    CLC_HAL_clearISR2InterruputFlag();
}

#endif

interrupt void ISR3(void)
{
    EINT;

    CLC_HAL_setProfilingGPIO3();
    CLC_runISR3();
    CLC_HAL_resetProfilingGPIO3();
    DINT;
    CLC_HAL_clearISR3InterruputFlag();
}

interrupt void sciaRxISR(void)
{
    uint16_t Accept_words=0;
    uint16_t CRCTest16, CRCTest16_Accept, CRCTest16_calc; //CRCTest8
    uint16_t uwtmp,uwtmp1,uwtmp2,uwtmp3;
    uint16_t Self_Addr = 0x01;
    int itmp = 0;

    EINT;

    Accept_words = SCI_getRxFIFOStatus(SCIA_BASE);

    for (uwtmp = 0;uwtmp < Accept_words; uwtmp++)
    {
    // Read two characters from the FIFO.
    receivedChar[uwtmp] = SCI_readCharBlockingFIFO(SCIA_BASE);
    }

    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
    DINT;  // Nest Interrupt
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    //uwcounter++;
}

interrupt void sciaTxISR(void)
{
    //CLC_HAL_setProfilingGPIO1();
    //CLC_HAL_resetProfilingGPIO1();

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
//=====================================================================
void Task_A0(void)  // Task_B is the same
{
    if(CLC_GET_TASKA_TIMER_OVERFLOW_STATUS == 1)
    {
        CLC_CLEAR_TASKA_TIMER_OVERFLOW_FLAG;    // clear flag
        // jump to an Task_A1 Task
        (*A_Task_Ptr)();
        //vTimer0[0]++;           // virtual timer 0, instance 0 (spare)
    }
}
void Task_A1(void)
{
    A_Task_Ptr = &Task_A1;
    static uint16_t countB1 = 0;

    #if CLC_SFRA_TYPE != CLC_SFRA_DISABLED
        CLC_runSFRABackGroundTasks();
    #endif

    //CLC_Print();  // Use SCI for Send out Interface
    CLC_updateBoardStatus();
    clc_HAL_toggleLED1();

    /*****   Start to Edit CAN GUI Interface    ************/

    if(send_En)
    {
        Can_Encode(txMsgData_OBC1, 0x572);
        Can_Encode(txMsgData_OBC2, 0x3D6);

        CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID_OBC1, 8, txMsgData_OBC1);
        CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID_OBC2, 8, txMsgData_OBC2);
    }

    if(uwObcPwrOnMdCmd == 0x01)
        DcChargSt_On = On;
    else
    {
        DcChargSt_On = OFF;
        PWMSta.PriPwmEnb_Act = 0U;
        PWMSta.SecPwmEnb_Act = 0U;
    }
    DcChargUnitSt_Cmd = (enum State_GUI)(uwObcCtrlMdCmd & 0x0F);
    /* Continue voSetCtrlMd to edit act on ISR3 */
}

void DAC_init(void)
{
    // Set VDAC as the DAC reference voltage.
    // Edit here to use ADC VREF as the reference voltage.
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_VDAC);
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_VDAC);
    DAC_setReferenceVoltage(DACC_BASE, DAC_REF_VDAC);

    // Enable the DAC output
    DAC_enableOutput(DACA_BASE);
    DAC_enableOutput(DACB_BASE);
    DAC_enableOutput(DACC_BASE);

    // Set the DAC shadow output to 0
    DAC_setShadowValue(DACA_BASE, 0);
    DAC_setShadowValue(DACB_BASE, 0);
    DAC_setShadowValue(DACC_BASE, 0);

    // Set VDAC as the DAC reference voltage.
    // Edit here to use ADC VREF as the reference voltage.
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_VDAC);
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_VDAC);
    DAC_setReferenceVoltage(DACC_BASE, DAC_REF_VDAC);

    // Enable the DAC output
    DAC_enableOutput(DACA_BASE);
    DAC_enableOutput(DACB_BASE);
    DAC_enableOutput(DACC_BASE);

    // Set the DAC shadow output to 0
    DAC_setShadowValue(DACA_BASE, 0);
    DAC_setShadowValue(DACB_BASE, 0);
    DAC_setShadowValue(DACC_BASE, 0);

    // Delay for buffered DAC to power up
    DEVICE_DELAY_US(10);
}

void clc_ADC_Offset_Calc(void)
{
    uint16_t itmp;

    float CLC_iPrimSenseOff_pu_sum, CLC_vPrimSenseOff_pu_sum, CLC_iSecSenseOff_pu_sum, CLC_vSecSenseOff_pu_sum;

    CLC_iPrimSenseOff_pu_sum = 0.0;
    CLC_vPrimSenseOff_pu_sum = 0.0;
    CLC_iSecSenseOff_pu_sum = 0.0;
    CLC_vSecSenseOff_pu_sum =0.0;

    DEVICE_DELAY_US(100);

    for(itmp = 0; itmp < 100; itmp++)
    {
        DEVICE_DELAY_US(15);

        CLC_iPrimSenseOff_pu_sum += (CLC_iPrimSense_pu*0.5f)+0.5f;
        CLC_iSecSenseOff_pu_sum += (CLC_iSecSense_pu*0.5f)+0.5f;
        CLC_vPrimSenseOff_pu_sum += CLC_vPrimSense_pu;
        CLC_vSecSenseOff_pu_sum += CLC_vSecSense_pu;
    }

    CLC_iPrimSenseOff_pu = CLC_iPrimSenseOff_pu_sum * 0.01f;
    CLC_iSecSenseOff_pu = CLC_iSecSenseOff_pu_sum * 0.01f;
    CLC_vPrimSenseOff_pu = CLC_vPrimSenseOff_pu_sum * 0.01f;
    CLC_vSecSenseOff_pu = CLC_vSecSenseOff_pu_sum * 0.01f;
}

//
// CAN A ISR - The interrupt service routine called when a CAN interrupt is
//             triggered on CAN module A.
interrupt void canaISR(void)
{
    uint32_t status,status1;
    bool BMS2_newData, VCU6_newData;

    status = CAN_getInterruptCause(CANA_BASE);
    if(status == CAN_INT_INT0ID_STATUS)
    {
        status1 = CAN_getStatus(CANA_BASE);
        if(((status1  >= CAN_STATUS_LEC_STUFF) & (status1  <= CAN_STATUS_LEC_MSK)) | (status1  == CAN_STATUS_PERR))
            errorFlag = 1;
        else if(status1 == CAN_STATUS_RXOK)
        {
            BMS2_newData = CAN_readMessage(CANA_BASE, RX_MSG_OBJ_ID_BMS2, rxMsgData_BMS);
            if(BMS2_newData)
                Can_Decode(rxMsgData_BMS, BMS2_newData, 0x376);

            VCU6_newData = CAN_readMessage(CANA_BASE, RX_MSG_OBJ_ID_VCU6, rxMsgData_VCU);

            if(VCU6_newData)
                Can_Decode(rxMsgData_VCU, VCU6_newData, 0x51F);

            CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID_BMS2);
            CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID_VCU6);
            CAN_clearInterruptStatus(CANA_BASE, CAN_INT_INT0ID_STATUS);
            rxMsgCount++;
            errorFlag = 0;
        }
        else if(status1 == CAN_STATUS_TXOK)
            CAN_clearInterruptStatus(CANA_BASE, CAN_INT_INT0ID_STATUS);
        else
            CAN_clearInterruptStatus(CANA_BASE, CAN_INT_INT0ID_STATUS);
    }
    else {
        errorFlag = 1;
    }

    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

void HAL_init()
{
    clc_initGlobalVar();

    Alpha_State_Ptr = &Task_A0;
    A_Task_Ptr = &Task_A1;

    //clc_BuildLvl_IndicaVar();
    clc_HAL_setDevice();

    // Stop all PWM mode clock
    CLC_HAL_disablePWMClkCounting();

    // set PWMs for the CLLC prim and sec bridges as battery charging mode
    clc_Hal_setPWM();

    // It is configured for ADC trig to AD fast signal, Up down mode.
    // It is trig by CMPA = 0,
    clc_Hal_setPWMinUpDownCountMode(CLC_ISR2_PWM_BASE, CLC_ISR2_FREQUENCY_HZ, CLC_PWMSYSCLOCK_FREQ_HZ);

    // ECAP is up mode, use 1/2 Period to trigger control, so First is Current sense and then perform ISR2
    CLC_HAL_setupECAPinPWMMode(CLC_ISR2_ECAP_BASE, CLC_ISR2_FREQUENCY_HZ, CLC_PWMSYSCLOCK_FREQ_HZ);

    // ECAP is up mode, use 1/4 Period to trigger control
    CLC_HAL_setupECAPinPWMMode_ISR3(CLC_ISR3_ECAP_BASE, CLC_ISR3_FREQUENCY_HZ, CLC_PWMSYSCLOCK_FREQ_HZ);

    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_ECAP1,  SYSCTL_SYNC_IN_SRC_EXTSYNCIN2);
    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_ECAP4,  SYSCTL_SYNC_IN_SRC_EXTSYNCIN2);
    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM10,  SYSCTL_SYNC_IN_SRC_EXTSYNCIN2);

    clc_HAL_setADC();

    // Profiling GPIO
    clc_HAL_setProfileGPIO();

    // configure LED GPIO
    clc_HAL_setLED();

    clc_HAL_setProtect();

    clc_HAL_setBdVoutProtect();  // Add Tz1 protect for Vout Over Voltage protect

    // setup trigger for the ADC conversions
    clc_HAL_setTrigForADC();

    // 2sec Synchronous Rectification On,OFF : OFF is Body Diode Rectification
#if(SynREC)
    clc_Hal_setPWMpins(pwmSwSta_SynchrusRectifi_Act);
#else
    clc_Hal_setPWMpins(SynchrosRectifi_OFF);
#endif

    clc_HAL_setSCI();

    // ISR Mapping
    CLC_HAL_setupInterrupt();

    // Enable PWM Clocks
    CLC_HAL_enablePWMClkCounting();

    clc_ADC_Offset_Calc();

    DAC_init();
}
