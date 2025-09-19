#include "clllc_hal.h"
//  self-define
#include "Ai_CLLLC_ADC_settings.h"
//
// Global variables used that are applicable to this board
// Global variable used by the SFO library
// Result can be used for all HRPWM channels
// This variable is also copied to HRMSTEP
// register by SFO(0) function.
int16_t MEP_ScaleFactor;

volatile uint32_t ePWM[9] = {0, EPWM1_BASE, EPWM2_BASE, EPWM3_BASE, EPWM4_BASE, EPWM5_BASE, EPWM6_BASE, EPWM7_BASE, EPWM8_BASE};

//  This routine sets up the basic device configuration such as initializing PLL
//  CPU timers and copying code from FLASH to RAM
void clc_HAL_setDevice(void)
{
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pull-ups.
    Device_initGPIO();

    // Initialize the CAN controllers
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);

    //GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);
    //GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);

    CAN_initModule(CANA_BASE);
    //CAN_initModule(CANB_BASE);

    // Set up the CAN bus bit rate to 500kHz for each module
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);
    //CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 1000000, 20);

    // Enable interrupts on the CAN A peripheral.
    CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);

    /* CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS); */

    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();

    // Initialize timer period to rate at which they will be used to slice of task
    CPUTimer_setPeriod(CLC_TASKA_CPUTIMER_BASE,DEVICE_SYSCLK_FREQ / CLC_TASKA_FREQ_HZ);
    CPUTimer_setPeriod(CLC_TASKB_CPUTIMER_BASE,DEVICE_SYSCLK_FREQ / CLC_TASKB_FREQ_HZ);

    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_setPreScaler(CLC_TASKA_CPUTIMER_BASE, 0);
    CPUTimer_setPreScaler(CLC_TASKB_CPUTIMER_BASE, 0);

    // Make sure timer is stopped
    CPUTimer_stopTimer(CLC_TASKA_CPUTIMER_BASE);
    CPUTimer_stopTimer(CLC_TASKB_CPUTIMER_BASE);

    CPUTimer_setEmulationMode(CLC_TASKA_CPUTIMER_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setEmulationMode(CLC_TASKB_CPUTIMER_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    CPUTimer_startTimer(CLC_TASKA_CPUTIMER_BASE);
    //CPUTimer_startTimer(CLC_TASKB_CPUTIMER_BASE);

    /***************************** CAN A module Initial ********************************/
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    // This registers the interrupt handler in PIE vector table.

    Interrupt_register(INT_CANA0, &canaISR);
    //Interrupt_register(INT_CANB0, &canbISR);  // CAN B cancel

    // Enable the CAN-A interrupt signal
    Interrupt_enable(INT_CANA0);
    //Interrupt_enable(INT_CANB0);

    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    //CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      CAN Module: A
    //      Message Object ID Number: 1
    //      Message Identifier: 0x31
    //      Message Frame: EXT
    //      Message Type: Transmit
    //      Message Identifier: 0x05555554
    //      Message Frame: Extended
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    //  to BMS
    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID_OBC1, 0x572, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);

    // to Broadcast
    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID_OBC2, 0x3D6, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);

    // BMS to OBC
    CAN_setupMessageObject(CANA_BASE, RX_MSG_OBJ_ID_BMS2, 0x376, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);

    // from BMS to Broadcast
    CAN_setupMessageObject(CANA_BASE, RX_MSG_OBJ_ID_VCU6, 0x51F, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);

    // Initialize the transmit message object data buffer to be sent
    txMsgData[0] = 0x12;
    txMsgData[1] = 0x34;
    txMsgData[2] = 0x56;
    txMsgData[3] = 0x78;
    txMsgData[4] = 0x9A;
    txMsgData[5] = 0xBC;
    txMsgData[6] = 0xDE;
    txMsgData[7] = 0xF0;

    CAN_startModule(CANA_BASE);
    //CAN_startModule(CANB_BASE);
}

#if(0)
void CLC_HAL_setupIprimSenseSignalChain(void)
{
    //
    // Set VDAC as the DAC reference voltage.
    // Edit here to use ADC VREF as the reference voltage.
    //
    DAC_setReferenceVoltage(CLC_IPRIM_PGA_REF_DAC_MODULE, DAC_REF_ADC_VREFHI);
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    DAC_setGainMode(CLC_IPRIM_PGA_REF_DAC_MODULE, DAC_GAIN_TWO);
    DAC_setGainMode(DACB_BASE, DAC_GAIN_TWO);

    //
    // Enable the DAC output
    //
    DAC_enableOutput(CLC_IPRIM_PGA_REF_DAC_MODULE);
    DAC_enableOutput(DACB_BASE);

    //
    // Set the DAC shadow output to 0
    //
    DAC_setShadowValue(CLC_IPRIM_PGA_REF_DAC_MODULE, 0);
    DAC_setShadowValue(DACB_BASE, 0);
    //
    // Delay for buffered DAC to power up
    //
    DEVICE_DELAY_US(10);

    //
    // Load shadow value into DAC
    //
    DAC_setShadowValue(CLC_IPRIM_PGA_REF_DAC_MODULE, 2048);
    DAC_setShadowValue(DACB_BASE, 2048);

    // Herman Disable
    // Set PGA gain
    //
    //PGA_setGain(CLC_IPRIM_PGA_MODULE, CLC_IPRIM_PGA_GAIN);

    //
    // No filter resistor for output
    //
    // PGA_setFilterResistor(CLC_IPRIM_PGA_MODULE, PGA_LOW_PASS_FILTER_DISABLED);

    //
    // disable PGA1
    //
    // PGA_enable(CLC_IPRIM_PGA_MODULE);
}
#endif

void clc_HAL_setProtect()
{
    // Disable all the muxes first
    XBAR_disableEPWMMux(XBAR_TRIP4, 0xFF);
/*
#if CLC_BOARD_PROTECTION_IPRIM == 1

    CLC_HAL_setupCMPSS(CLC_IPRIM_CMPSS_BASE, CLC_IPRIM_TRIP_LIMIT_AMPS, CLC_IPRIM_MAX_SENSE_AMPS);

    XBAR_setEPWMMuxConfig(XBAR_TRIP4, CLC_IPRIM_XBAR_MUX_VAL);
    XBAR_enableEPWMMux(XBAR_TRIP4, CLC_IPRIM_XBAR_MUX);

    XBAR_clearInputFlag(CLC_IPRIM_CMPSS_XBAR_FLAG1);
    XBAR_clearInputFlag(CLC_IPRIM_CMPSS_XBAR_FLAG2);
#else
    #warning BOARD_PROTECTION_IPRIM is disabled
#endif
*/
#if CLC_BOARD_PROTECTION_IPRIM_TANK == 1

    clc_Hal_setIPRMtankCmpss_HighLowLim(CLC_IPRIM_SHUNT_CMPSS_BASE, CLC_IPRIM_TANK_TRIP_LIMIT_AMPS, CLC_IPRIM_TANK_MAX_SENSE_AMPS,
                                        CLC_CMPSS_HYSTERESIS, CLC_CMPSSS_FILTER_PRESCALAR, CLC_CMPSS_WINODW, CLC_CMPSS_THRESHOLD);

    XBAR_setEPWMMuxConfig(XBAR_TRIP4, CLC_IPRIM_SHUNT_CMPSS_XBAR_MUX_VAL);
    XBAR_enableEPWMMux(XBAR_TRIP4, CLC_IPRIM_SHUNT_CMPSS_XBAR_MUX);
    XBAR_clearInputFlag(CLC_IPRIM_SHUNT_CMPSS_XBAR_FLAG1);
    XBAR_clearInputFlag(CLC_IPRIM_SHUNT_CMPSS_XBAR_FLAG2);
#else
    #warning BOARD_PROTECTION_IPRIM_TANK is disabled
#endif

#if CLC_BOARD_PROTECTION_IPRIM_TANK == 1 || CLC_BOARD_PROTECTION_VSEC == 1 || CLC_BOARD_PROTECTION_ISEC == 1 || CLC_BOARD_PROTECTION_VPRIM == 1

    //Trip 4 is the input to the DCAHCOMPSEL
    EPWM_selectDigitalCompareTripInput(CLC_PRIM_LEG1_PWM_BASE, EPWM_DC_TRIP_TRIPIN4, EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);
    EPWM_setDigitalCompareEventSource(CLC_PRIM_LEG1_PWM_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
    EPWM_setDigitalCompareEventSyncMode(CLC_PRIM_LEG1_PWM_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_NOT_SYNCED);

    EPWM_selectDigitalCompareTripInput(CLC_SEC_LEG1_PWM_BASE, EPWM_DC_TRIP_TRIPIN4, EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);
    EPWM_setDigitalCompareEventSource(CLC_SEC_LEG1_PWM_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
    EPWM_setDigitalCompareEventSyncMode(CLC_SEC_LEG1_PWM_BASE, EPWM_DC_MODULE_A , EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_NOT_SYNCED);

    // Enable the following trips Emulator Stop, TZ1-3 and DCAEVT1
    EPWM_enableTripZoneSignals(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_SIGNAL_DCAEVT1);
    EPWM_enableTripZoneSignals(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_SIGNAL_DCAEVT1);

    // Enable the following trips Emulator Stop
    //
    EPWM_enableTripZoneSignals(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_SIGNAL_CBC6);
    EPWM_enableTripZoneSignals(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_SIGNAL_CBC6);

    EPWM_enableTripZoneSignals(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_SIGNAL_OSHT1);
    EPWM_enableTripZoneSignals(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_SIGNAL_OSHT1);

#else
    #warning All current comparator based protections are disabled
#endif

    // TZA events can force EPWMxA
    // TZB events can force EPWMxB
    EPWM_setTripZoneAction(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);

    EPWM_clearCycleByCycleTripZoneFlag(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_CBC_FLAG_6);
    EPWM_clearCycleByCycleTripZoneFlag(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_CBC_FLAG_6);

    // Clear any spurious trip
    EPWM_clearTripZoneFlag(CLC_PRIM_LEG1_PWM_BASE, (EPWM_TZ_INTERRUPT_OST | EPWM_TZ_INTERRUPT_DCAEVT1));
    EPWM_clearTripZoneFlag(CLC_SEC_LEG1_PWM_BASE, (EPWM_TZ_INTERRUPT_OST | EPWM_TZ_INTERRUPT_DCAEVT1));

    EPWM_forceTripZoneEvent(CLC_PRIM_LEG1_PWM_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(CLC_SEC_LEG1_PWM_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

void clc_HAL_setBdVoutProtect(void)
{
#if CLC_BOARD_PROTECTION_VSEC

    GPIO_setDirectionMode(56, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(56, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_56_GPIO56);
    // Configure Input-XBAR INPUT1 to GPIO12
    XBAR_setInputPin(XBAR_INPUT1, 56);

#else

#endif
}

void clc_HAL_setTrigForADC(void)
{
    EPWM_setADCTriggerSource(CLC_ISR2_PWM_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPB);

    // set CMPB such that the triggers for the ADC are centered around zero
    // of the ISR generating timebase
    EPWM_setCounterCompareValue(CLC_ISR2_PWM_BASE, EPWM_COUNTER_COMPARE_B,(CLC_IPRIM_TANK_ADC_ACQPS_SYS_CLKS * ((float32_t)CLC_PWMSYSCLOCK_FREQ_HZ / (float32_t)CLC_CPU_SYS_CLK_FREQ_HZ)));

    // Generate pulse on 1st even
    EPWM_setADCTriggerEventPrescale(CLC_ISR2_PWM_BASE, EPWM_SOC_A, 1);

    // Enable SOC on A group
    EPWM_enableADCTrigger(CLC_ISR2_PWM_BASE, EPWM_SOC_A);
}

void clc_HAL_setLED(void)
{
    GPIO_setDirectionMode(CLC_GPIO_LED1, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(CLC_GPIO_LED1, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(CLC_GPIO_LED1_PIN_CONFIG);

    GPIO_setDirectionMode(CLC_GPIO_LED_BL, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(CLC_GPIO_LED_BL, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(CLC_GPIO_LED_BL_PIN_CONFIG);

    GPIO_setDirectionMode(CLC_GPIO_LED_ORG, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(CLC_GPIO_LED_ORG, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(CLC_GPIO_LED_ORG_PIN_CONFIG);
}

void clc_HAL_toggleLED1(void)
{
    static uint16_t ledCnt1 = 0;

    if(ledCnt1 == 0)
    {
        GPIO_togglePin(CLC_GPIO_LED1);
        ledCnt1 = 1;
    }
    else
        ledCnt1--;
}

void clc_HAL_setProfileGPIO(void)
{
    GPIO_setDirectionMode(CLC_GPIO_PROFILING1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(CLC_GPIO_PROFILING2, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(CLC_GPIO_PROFILING3, GPIO_DIR_MODE_OUT);
//    GPIO_setDirectionMode(CLC_GPIO_PWMEnable, GPIO_DIR_MODE_OUT);
//    GPIO_setDirectionMode(CLC_GPIO_PWMEnable_SR, GPIO_DIR_MODE_OUT);

    GPIO_setQualificationMode(CLC_GPIO_PROFILING1, GPIO_QUAL_SYNC);
    GPIO_setQualificationMode(CLC_GPIO_PROFILING2, GPIO_QUAL_SYNC);
    GPIO_setQualificationMode(CLC_GPIO_PROFILING3, GPIO_QUAL_SYNC);
//    GPIO_setQualificationMode(CLC_GPIO_PWMEnable, GPIO_QUAL_SYNC);
//    GPIO_setQualificationMode(CLC_GPIO_PWMEnable_SR, GPIO_QUAL_SYNC);

    GPIO_setPinConfig(CLC_GPIO_PROFILING1_PIN_CONFIG);
    GPIO_setPinConfig(CLC_GPIO_PROFILING2_PIN_CONFIG);
    GPIO_setPinConfig(CLC_GPIO_PROFILING3_PIN_CONFIG);
//    GPIO_setPinConfig(CLC_GPIO_PROFILING4_PIN_CONFIG);
//    GPIO_setPinConfig(CLC_GPIO_PROFILING5_PIN_CONFIG);

}

void clc_HAL_setSCI(void)
{
    // setup Gpio for SCI comms for SFRA
    GPIO_setPinConfig(GPIO_28_SCIRXDA);   //GPIO_28_SCIRXDA  GPIO_17_SCIRXDA
    GPIO_setPinConfig(GPIO_29_SCITXDA);   //GPIO_29_SCITXDA  GPIO_16_SCITXDA
    GPIO_setQualificationMode(28, GPIO_QUAL_ASYNC);  // 28
    GPIO_setQualificationMode(29, GPIO_QUAL_ASYNC);  // 29
    EDIS;
#if(0)
    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);
#endif

    // Disable global interrupts.
    DINT;

    // Initialize interrupt controller and vector table.
    Interrupt_register(INT_SCIA_TX, sciaTxISR);
    Interrupt_register(INT_SCIA_RX, sciaRxISR);

    // Initialize SCIA and its FIFO.
    SCI_performSoftwareReset(SCIA_BASE);

    // 50000000 is the LSPCLK or the Clock used for the SCI Module
    // 57600 is the Baudrate desired of the SCI module
    // 1 stop bit,  No parity, 8 char bits,
    SCI_setConfig(SCIA_BASE, 50000000, 115200,  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));//    org is 115200 57600

    //No loopback
    //SCI_enableLoopback(SCIA_BASE);

    SCI_resetChannels(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);

    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);

    // Set the transmit FIFO level to 0 and the receive FIFO level to 2.
    // Enable the TXFF and RXFF interrupts.
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX6);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF);
    // SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    // SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    // SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXRDY_BRKDT  | SCI_INT_RXFF | SCI_INT_RXERR);
    // Disable RX ERR, SLEEP, TXWAKE

    Interrupt_enable(INT_SCIA_RX);
    //Interrupt_enable(INT_SCIA_TX);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

// the following routines sends the command packet to the PFC stage
// the command packet is organized as follows
// /r/n{Mode}{Voltage ref}
// in total 4 hex values are transmitted ,
// first two are /r and /n which signify start of packet
// next is a hex ASCII for P or I depending on the mode ,
// if mode is 0 that is PFC/Inveretr is off O is transmitted
// if mode is 1 that is PFC then P is transmitted
// if mode is 2 that is Inverter I is transmitted
// the next value is hex that represents the voltage reference
// the hex value is determined as follows (voltage_ref - 400 +32)
// +30 is done to avoid special hex characters
// -400 is done to keep the range of this message in 8 bits
#pragma FUNC_ALWAYS_INLINE(clc_HAL_sendCmdOverSci)
#pragma FUNC_ALWAYS_INLINE(SCI_writeCharBlockingFIFO)
void clc_HAL_sendCmdOverSci(uint16_t mode, uint16_t voltage_ref)
{
    // below is usefull for debug
    // return, 13 decimal
    //SCI_writeCharBlockingFIFO(SCIA_BASE, 13);
    // new line, 10 decimal
    //SCI_writeCharBlockingFIFO(SCIA_BASE, 10);

    if(mode == 2)   // Inverter Mode I-73 decimal
        SCI_writeCharBlockingFIFO(SCIA_BASE, 2);
    else if(mode == 1)  // PFC mode P - 80 decimal
        SCI_writeCharBlockingFIFO(SCIA_BASE, 1);
    else    // OFF 0 - 80 decimal
        SCI_writeCharBlockingFIFO(SCIA_BASE, 0);

    SCI_writeCharBlockingFIFO(SCIA_BASE, voltage_ref - 400 + 32);
}

// mode is 0: no PWM
// mode is 1: prim PWM on sec PWM off
// mode is 2: prim and sec PWM on
// mode is 3: prim and sec PWM on
void clc_Hal_setPWMpins(uint16_t mode)
{
    // if mode is 0 then disable prim & sec PWMs
    if(mode == 0)
    {
        GPIO_writePin(CLC_PRIM_LEG1_PWM_H_GPIO, 0);
        GPIO_writePin(CLC_PRIM_LEG1_PWM_L_GPIO, 0);
        GPIO_setPinConfig(CLC_PRIM_LEG1_PWM_H_DIS_GPIO_PIN_CONFIG);
        GPIO_setPinConfig(CLC_PRIM_LEG1_PWM_L_DIS_GPIO_PIN_CONFIG);
    }

    // if mode is 0 or 1 then disable sec PWM
    if(mode == 0 || mode == 1)
    {
        GPIO_writePin(CLC_SEC_LEG1_PWM_H_GPIO, 0);
        GPIO_writePin(CLC_SEC_LEG1_PWM_L_GPIO, 0);
        GPIO_setPinConfig(CLC_SEC_LEG1_PWM_H_DIS_GPIO_PIN_CONFIG);
        GPIO_setPinConfig(CLC_SEC_LEG1_PWM_L_DIS_GPIO_PIN_CONFIG);
    }

    // if mode is 1 or 2 or 3 then enable prim PWM
    if(mode == 1 || mode == 2 || mode == 3)
    {
        GPIO_setDirectionMode(CLC_PRIM_LEG1_PWM_H_GPIO, GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(CLC_PRIM_LEG1_PWM_H_GPIO, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(CLC_PRIM_LEG1_PWM_H_GPIO_PIN_CONFIG);

        GPIO_setDirectionMode(CLC_PRIM_LEG1_PWM_L_GPIO, GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(CLC_PRIM_LEG1_PWM_L_GPIO, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(CLC_PRIM_LEG1_PWM_L_GPIO_PIN_CONFIG);
    }

    // if mode is 2 or 3 then enable sec PWM
    if(mode == 2 || mode == 3)
    {

        GPIO_setDirectionMode(CLC_SEC_LEG1_PWM_L_GPIO, GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(CLC_SEC_LEG1_PWM_L_GPIO, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(CLC_SEC_LEG1_PWM_L_GPIO_PIN_CONFIG);

        GPIO_setDirectionMode(CLC_SEC_LEG1_PWM_H_GPIO, GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(CLC_SEC_LEG1_PWM_H_GPIO, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(CLC_SEC_LEG1_PWM_H_GPIO_PIN_CONFIG);
    }
}

void clc_Hal_setCLA(void)
{
#if CLC_ISR1_RUNNING_ON == CLA_CORE

    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
            (uint32_t)&Cla1ProgLoadSize );
    // Mark This memcpy will no effect

    //
    // first assign memory to CLA
    //
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS2, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS3, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMMASTER_CPU_CLA1);

    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS2, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS3, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5, MEMCFG_CLA_MEM_PROGRAM);

    //
    // Suppressing #770-D conversion from pointer to smaller integer
    // The CLA address range is 16 bits so the addresses passed to the MVECT
    // registers will be in the lower 64KW address space. Turn the warning
    // back on after the MVECTs are assigned addresses
    //
    #pragma diag_suppress = 770

    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_1, (uint16_t)&Cla1Task1);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_2, (uint16_t)&Cla1Task2);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_3, (uint16_t)&Cla1Task3);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_4, (uint16_t)&Cla1Task4);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_5, (uint16_t)&Cla1Task5);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_6, (uint16_t)&Cla1Task6);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_7, (uint16_t)&Cla1Task7);
    CLA_mapBackgroundTaskVector(CLA1_BASE, (uint16_t)&Cla1BackgroundTask);

    #pragma diag_warning = 770

    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_ALL);

    CLA_enableHardwareTrigger(CLA1_BASE);
    CLA_setTriggerSource(CLA_TASK_8, CLC_ISR2_TRIG_CLA);
    CLA_enableBackgroundTask(CLA1_BASE);

    CLA_setTriggerSource(CLA_TASK_1, CLC_ISR1_TRIG_CLA);
#endif
}

void clc_Hal_setPWM(void)
{
    uint16_t status;

    //setup the prim PWM
    clc_Hal_setHRpwmUpDownCountModeWithDeadBand(CLC_PRIM_LEG1_PWM_BASE, CLC_NOMINAL_PWM_SWITCH_FREQ_HZ, CLC_PWMSYSCLOCK_FREQ_HZ,
                                                CLC_PRIM_PWM_DEADBAND_RED_NS, CLC_PRIM_PWM_DEADBAND_FED_NS);

    CLC_HAL_setupHRPWMinUpDownCount2ChAsymmetricMode(CLC_SEC_LEG1_PWM_BASE, CLC_NOMINAL_PWM_SWITCH_FREQ_HZ, CLC_PWMSYSCLOCK_FREQ_HZ);

    EPWM_disablePhaseShiftLoad(CLC_PRIM_LEG1_PWM_BASE);

    EPWM_setSyncOutPulseMode(CLC_PRIM_LEG1_PWM_BASE,EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    EPWM_enablePhaseShiftLoad(CLC_SEC_LEG1_PWM_BASE);
    EPWM_setSyncOutPulseMode(CLC_SEC_LEG1_PWM_BASE,EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

    EPWM_setPhaseShift(CLC_SEC_LEG1_PWM_BASE, 0);
    EPWM_setCountModeAfterSync(CLC_SEC_LEG1_PWM_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    // After trig Syn output pulse, the count mode is up by default EPWM3 syncout is passed to EPWM4

    //Enable swap deadband

    HWREGH(CLC_SEC_LEG1_PWM_BASE + EPWM_O_DBCTL) = (HWREGH(CLC_SEC_LEG1_PWM_BASE + EPWM_O_DBCTL) | 0x3000);

    EPWM_setupEPWMLinks(CLC_SEC_LEG1_PWM_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);
    EPWM_setupEPWMLinks(CLC_SEC_LEG1_PWM_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);

    EPWM_setGlobalLoadOneShotLatch(CLC_PRIM_LEG1_PWM_BASE);
}

void clc_Hal_setCMPSSHighLowLimit(uint32_t base1, float32_t currentLimit, float32_t currentMaxSense, uint16_t hysteresis,
                                  uint16_t filterClkPrescalar, uint16_t filterSampleWindow, uint16_t filterThreshold)
{
    //Enable CMPSS1
    CMPSS_enableModule(base1);

    //Use VDDA as the reference for comparator DACs
    CMPSS_configDAC(base1, CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);

    //Set DAC to H~75% and L ~25% values
    CMPSS_setDACValueHigh(base1, (int16_t)(2048.0 + (float)currentLimit * (float)2048.0 / (float)currentMaxSense));
    CMPSS_setDACValueLow(base1, (int16_t)(2048.0 - (float)currentLimit * (float)2048.0 / (float)currentMaxSense));

    // Make sure the asynchronous path compare high and low event
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator oputput is "not" inverted for high compare event
    CMPSS_configHighComparator(base1, CMPSS_INSRC_DAC);

    // Comparator output is inverted for for low compare event
    CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);
    //CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC);

    // The following sets the digital filter for the trip flag for high and low
    // the below configuration will buffer last 10 samples and flag a trip if
    // 7 of them are high
    CMPSS_configFilterHigh(base1, filterClkPrescalar, filterSampleWindow, filterThreshold);
    CMPSS_configFilterLow(base1, filterClkPrescalar, filterSampleWindow, filterThreshold);

    //Reset filter logic & start filtering
    CMPSS_initFilterHigh(base1);
    CMPSS_initFilterLow(base1);

    // Configure CTRIPOUT path
    CMPSS_configOutputsHigh(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);

    //Comparator hysteresis control , set to 2x typical value
    CMPSS_setHysteresis(base1, hysteresis);

    // Clear the latched comparator events
    CMPSS_clearFilterLatchHigh(base1);
    CMPSS_clearFilterLatchLow(base1);
}

void clc_Hal_setIPRMtankCmpss_HighLowLim(uint32_t base1, float32_t currentLimit, float32_t currentMaxSense, uint16_t hysteresis,
                                         uint16_t filterClkPrescalar, uint16_t filterSampleWindow, uint16_t filterThreshold)
{
    //Enable CMPSS1
    CMPSS_enableModule(base1);

    //Use VDDA as the reference for comparator DACs
    CMPSS_configDAC(base1, CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);

    //Set DAC to H~75% and L ~25% values
    CMPSS_setDACValueHigh(base1, (int16_t)((float)currentLimit * (float)4096.0 / (float)currentMaxSense));
    CMPSS_setDACValueLow(base1, (int16_t)((float)currentLimit * (float)4096.0 / (float)currentMaxSense));

    // Make sure the asynchronous path compare high and low event
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator oputput is "not" inverted for high compare event
    CMPSS_configHighComparator(base1, CMPSS_INSRC_DAC);

    // Comparator output is inverted for for low compare event
    //CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);
    CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC);

    // The following sets the digital filter for the trip flag for high and low
    // the below configuration will buffer last 10 samples and flag a trip if
    // 7 of them are high
    CMPSS_configFilterHigh(base1, filterClkPrescalar, filterSampleWindow, filterThreshold);
    CMPSS_configFilterLow(base1, filterClkPrescalar, filterSampleWindow, filterThreshold);

    //Reset filter logic & start filtering
    CMPSS_initFilterHigh(base1);
    CMPSS_initFilterLow(base1);

    // Configure CTRIPOUT path
    CMPSS_configOutputsHigh(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);

    //Comparator hysteresis control , set to 2x typical value
    CMPSS_setHysteresis(base1, hysteresis);

    // Clear the latched comparator events
    CMPSS_clearFilterLatchHigh(base1);
    CMPSS_clearFilterLatchLow(base1);
}

void clc_Hal_setCmpss_HighLowVoltLim(uint32_t base1, float32_t VoltLimit, float32_t VoltLowLimit, float32_t voltMaxSense,
                                     uint16_t hysteresis, uint16_t filterClkPrescalar, uint16_t filterSampleWindow, uint16_t filterThreshold)
{
    //Enable CMPSS1
    CMPSS_enableModule(base1);

    //Use VDDA as the reference for comparator DACs
    CMPSS_configDAC(base1, CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);

    //Set DAC to H~75% and L ~25% values
    CMPSS_setDACValueHigh(base1, 0 + (int16_t)((float)VoltLimit * (float)4096.0 / (float)voltMaxSense));
#if(VinLVProt)
    CMPSS_setDACValueLow(base1, 0 + (int16_t)((float)VoltLowLimit * (float)4096.0 / (float)voltMaxSense));
#else
    CMPSS_setDACValueLow(base1, 0);
#endif
    // Make sure the asynchronous path compare high and low event
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator oputput is "not" inverted for high compare event
    CMPSS_configHighComparator(base1, CMPSS_INSRC_DAC);

    // Comparator output is inverted for for low compare event
    CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);

    // The following sets the digital filter for the trip flag for high and low
    // the below configuration will buffer last 10 samples and flag a trip if
    // 7 of them are high
    CMPSS_configFilterHigh(base1, filterClkPrescalar, filterSampleWindow, filterThreshold);
    CMPSS_configFilterLow(base1, filterClkPrescalar, filterSampleWindow, filterThreshold);

    //Reset filter logic & start filtering
    CMPSS_initFilterHigh(base1);
    CMPSS_initFilterLow(base1);

    // Configure CTRIPOUT path
    CMPSS_configOutputsHigh(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);

    //Comparator hysteresis control , set to 2x typical value
    CMPSS_setHysteresis(base1, hysteresis);

    // Clear the latched comparator events
    CMPSS_clearFilterLatchHigh(base1);
    CMPSS_clearFilterLatchLow(base1);
}

void CLC_HAL_disablePWMClkCounting(void)
{
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

void CLC_HAL_enablePWMClkCounting(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

void clc_Hal_setPWMinUpDownCountMode(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz)
{

    uint32_t pwmPeriod_ticks;

    pwmPeriod_ticks = TICKS_IN_PWM_FREQUENCY(pwmFreq_Hz, pwmSysClkFreq_Hz);

    // Time Base SubModule Registers
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(base1, pwmPeriod_ticks >> 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // configure PWM 1 as master and Phase 2 as slaves and
    // let it pass the sync in pulse from PWM1
    EPWM_disablePhaseShiftLoad(base1);
    EPWM_setSyncOutPulseMode(base1, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    // Counter Compare Submodule Registers
    // set duty 0% initially
    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // Action Qualifier SubModule Registers
    HWREGH(base1 + EPWM_O_AQCTLA) = 0 ;
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

}


void CLC_HAL_setupECAPinPWMMode(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz)
{
    int32_t ecap_ticks = (int32_t) ((float32_t)(pwmSysClkFreq_Hz / pwmFreq_Hz));

    ECAP_enableAPWMMode(base1);
    ECAP_setAPWMPeriod(base1, ecap_ticks);
    ECAP_setAPWMCompare(base1, ecap_ticks >> 1);

    ECAP_setAPWMShadowPeriod(base1, ecap_ticks);
    ECAP_setAPWMShadowCompare(base1, ecap_ticks >> 1);

    //ECAP_setAPWMPolarity(base1,ECAP_APWM_ACTIVE_LOW);
    ECAP_setSyncOutMode(base1,ECAP_SYNC_OUT_SYNCI);

    //ECAP_setPhaseShiftCount(base1, PWM_PHASE_VAL);
    ECAP_setPhaseShiftCount(base1, 0U);
    ECAP_enableLoadCounter(base1);

    ECAP_clearInterrupt(base1, 0xFF);
    ECAP_clearGlobalInterrupt(base1);

    ECAP_startCounter(base1);

}

void CLC_HAL_setupECAPinPWMMode_ISR3(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz)
{
    int32_t ecap_ticks = (int32_t) ((float32_t)(pwmSysClkFreq_Hz / pwmFreq_Hz));

    ECAP_enableAPWMMode(base1);
    ECAP_setAPWMPeriod(base1, ecap_ticks);
    //ECAP_setAPWMCompare(base1, ecap_ticks>>5); // 231001 orig set
    ECAP_setAPWMCompare(base1, 100U);

    ECAP_setAPWMShadowPeriod(base1, ecap_ticks);
	//ECAP_setAPWMShadowCompare(base1, ecap_ticks>>5); // 231001 orig set
    ECAP_setAPWMShadowCompare(base1, 100U); // 231109 orig set

    //ECAP_setAPWMPolarity(base1,ECAP_APWM_ACTIVE_LOW);
	ECAP_setSyncOutMode(base1,ECAP_SYNC_OUT_SYNCI);

    //ECAP_setPhaseShiftCount(base1, PWM_PHASE_VAL);
    ECAP_setPhaseShiftCount(base1, 0U);
    ECAP_enableLoadCounter(base1);

    ECAP_clearInterrupt(base1, 0xFF);
    ECAP_clearGlobalInterrupt(base1);

    ECAP_startCounter(base1);
}

void clc_Hal_setHRpwmUpDownCountModeWithDeadBand(uint32_t base1, float32_t pwmFreq_Hz,
                                                 float32_t pwmSysClkFreq_Hz, float32_t red_ns, float32_t fed_ns)
{
    uint32_t pwmPeriod_ticks;
    uint32_t dbFED_ticks, dbRED_ticks;

    pwmPeriod_ticks = (uint32_t)((pwmSysClkFreq_Hz * (float32_t)TWO_RAISED_TO_THE_POWER_SIXTEEN) / (float32_t)pwmFreq_Hz) >> 1;
    pwmPeriod_ticks = (pwmPeriod_ticks & 0xFFFFFF00);

    dbRED_ticks = ((uint32_t)(red_ns * (float32_t)TWO_RAISED_TO_THE_POWER_SIXTEEN * ((float32_t)ONE_NANO_SEC) * pwmSysClkFreq_Hz * 2.0f));
    dbRED_ticks = (dbRED_ticks & 0xFFFFFE00);

    dbFED_ticks = ((uint32_t)(red_ns * (float32_t)TWO_RAISED_TO_THE_POWER_SIXTEEN * ((float32_t)ONE_NANO_SEC) * pwmSysClkFreq_Hz * 2.0f));
    dbFED_ticks = (dbFED_ticks & 0xFFFFFE00);

    // Time Base SubModule Registers
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_SHADOW_LOAD);
    HWREG(base1 + HRPWM_O_TBPRDHR) = pwmPeriod_ticks;

    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // Counter Compare Submodule Registers
    // set duty 50% initially
    HWREG(base1 + HRPWM_O_CMPA) = pwmPeriod_ticks >> 1;

    // set as shadow mode
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);

    // set as shadow mode
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);

    HWREG(base1 + HRPWM_O_CMPB) = pwmPeriod_ticks >> 1;

    EPWM_disableCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_C);

    EALLOW;

    // Clear AQCTLA, B and Deadband settings settings
    HWREGH(base1 + EPWM_O_AQCTLA) = 0x0000;
    HWREGH(base1 + EPWM_O_AQCTLB) = 0x0000;
    HWREGH(base1 + EPWM_O_DCBCTL) = 0x0000;
    EDIS;

    // Action Qualifier SubModule Registers
    // CTR = CMPA@UP , xA set to 1
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    // CTR = CMPA@Down , xA set to 0
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    // Active high complementary PWMs - Set up the deadband
    EPWM_setRisingEdgeDelayCountShadowLoadMode(base1, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(base1, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandCounterClock(base1, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);

    EPWM_setDeadBandDelayMode(base1, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base1, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDeadBandDelayInput(base1, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base1, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(base1, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayPolarity(base1, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);

    HWREG(base1 + HRPWM_O_DBFEDHR) = dbFED_ticks;
    HWREG(base1 + HRPWM_O_DBREDHR) = dbRED_ticks;

    // Hi-res PWM
    // MEP control on both edges.
    HRPWM_setMEPEdgeSelect(base1, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setCounterCompareShadowLoadEvent(base1, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);
    HRPWM_setMEPEdgeSelect(base1, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setCounterCompareShadowLoadEvent(base1, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);

    HRPWM_setMEPControlMode(base1, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(base1, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);

    HRPWM_setDeadbandMEPEdgeSelect(base1, HRPWM_DB_MEP_CTRL_RED_FED);

    // Enable autoconversion
    HRPWM_enableAutoConversion(base1);

    // Turn on high-resolution period control.
    HRPWM_enablePeriodControl(base1);


}

void CLC_HAL_setupHRPWMinUpDownCount2ChAsymmetricMode(uint32_t base1, float32_t pwmFreq_Hz, float32_t pwmSysClkFreq_Hz)
{
    uint32_t pwmPeriod_ticks;

    pwmPeriod_ticks = TICKS_IN_PWM_FREQUENCY(pwmFreq_Hz, pwmSysClkFreq_Hz);

    // Time Base SubModule Registers
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBasePeriod(base1, pwmPeriod_ticks >> 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // Counter Compare Submodule Registers
    // set duty 50% initially
//    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A, pwmPeriod_ticks >> 2);  //2, It should be fix 1.0us,  or looktable
    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A, pwmPeriod_ticks >> 2);  //2, It should be fix 0.9us,  or looktable

    // set as shadow mode
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);

    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_B, pwmPeriod_ticks >> 2);  //2

    EALLOW;

    // Clear AQCTLA, B and Deadband settings settings
    HWREGH(base1 + EPWM_O_AQCTLA) = 0x0000;
    HWREGH(base1 + EPWM_O_AQCTLB) = 0x0000;

    HWREGH(base1 + EPWM_O_DCBCTL) = 0x0000;
    EDIS;

    // Action Qualifier SubModule Registers
    // CTR = CMPA@0 , xA set to 1
     EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO );

    // CTR = CMPA@Up , xA set to 0

    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);  // EPWM_AQ_OUTPUT_A is PWM3B

    // CTR = CMPB@PERIOD, xB set to 1
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    // CTR = CMPB@DOWN , xB set to 0
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    // Hi-res PWM
    // MEP control on both edges.
    HRPWM_setMEPEdgeSelect(base1, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setCounterCompareShadowLoadEvent(base1, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);
    HRPWM_setMEPEdgeSelect(base1, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setCounterCompareShadowLoadEvent(base1, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);

    HRPWM_setMEPControlMode(base1, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(base1, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);

    // Enable autoconversion
    HRPWM_enableAutoConversion(base1);

    // Enable TBPHSHR sync (required for updwn count HR control)
    HRPWM_enablePhaseShiftLoad(base1);

    // Turn on high-resolution period control.
    HRPWM_enablePeriodControl(base1);
}

void CLC_setSynOut(void)
{
        // For SynC out set X-BAR Mux 14 to XBAR_Output4

        XBAR_setOutputMuxConfig(XBAR_OUTPUT4, CLC_SynCout_XBAR_MUX_VAL);  //CLC_SynCout_XBAR_MUX_VAL
        XBAR_enableOutputMux(XBAR_OUTPUT4, CLC_SynCout);

        GPIO_setDirectionMode(CLC_GPIO_XBAR4, GPIO_DIR_MODE_OUT);

        GPIO_setQualificationMode(CLC_GPIO_XBAR4, GPIO_QUAL_SYNC);
        GPIO_setPinConfig(CLC_GPIO_XBAR4_PIN_CONFIG);
}

void CLC_setSynIn(void)
{
    EALLOW;

    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(40, GPIO_QUAL_SYNC);

    GPIO_setPinConfig(GPIO_40_GPIO40);
    //
    // Configure Input-XBAR INPUT5 to GPIO40
    XBAR_setInputPin(XBAR_INPUT1, 10);  // trouble in SR_use_Absolute_curr == 0
    XBAR_setInputPin(XBAR_INPUT2, 10);  // trouble in SR_use_Absolute_curr == 0
    XBAR_setInputPin(XBAR_INPUT3, 10);  // trouble in SR_use_Absolute_curr == 0
    XBAR_setInputPin(XBAR_INPUT4, 10);  // trouble in SR_use_Absolute_curr == 0
    XBAR_setInputPin(XBAR_INPUT5, 10);  // trouble in SR_use_Absolute_curr == 0
    XBAR_setInputPin(XBAR_INPUT6, 10);  // trouble in SR_use_Absolute_curr == 0

    EDIS;

}

void CLC_HAL_setupCMPSS(uint32_t base1, float32_t current_limit, float32_t current_max_sense)
{
    // Enable CMPSS1
    CMPSS_enableModule(base1);

    // Use VDDA as the reference for comparator DACs
    CMPSS_configDAC(base1, CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);

    // Set DAC to H~75% and L ~25% values
    CMPSS_setDACValueHigh(base1, 2048 + (int16_t)((float32_t)current_limit * (float32_t)2048.0f / (float32_t)current_max_sense));
    CMPSS_setDACValueLow(base1, 2048 - (int16_t)((float32_t)current_limit * (float32_t)2048.0f / (float32_t)current_max_sense));

    // Make sure the asynchronous path compare high and low event
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator oputput is "not" inverted for high compare event
    CMPSS_configHighComparator(base1, CMPSS_INSRC_DAC);

    // Comparator output is inverted for for low compare event
    CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);

    CMPSS_configFilterHigh(base1, 2, 30, 20); //10 to 30; 7 to 20
    CMPSS_configFilterLow(base1, 2, 30, 20);

    // Reset filter logic & start filtering
    CMPSS_initFilterHigh(base1);
    CMPSS_initFilterLow(base1);

    // Configure CTRIPOUT path
    CMPSS_configOutputsHigh(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);

    // Comparator hysteresis control , set to 2x typical value
    CMPSS_setHysteresis(base1, 2);

    // Clear the latched comparator events
    CMPSS_clearFilterLatchHigh(base1);
    CMPSS_clearFilterLatchLow(base1);
}
