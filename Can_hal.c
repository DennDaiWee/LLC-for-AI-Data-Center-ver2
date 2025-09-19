#ifndef _CAN_HAL_C
    #define _CAN_HAL_C
#endif

#include "driverlib.h"
#include "device.h"
#include "Data.h"

#define TXCOUNT  2500
#define MSG_DATA_LENGTH    8

#define RX_MSG_OBJ_ID_VCU6   2

#define TX_MSG_OBJ_ID_OBC1   10  // from OBC to BMS
#define TX_MSG_OBJ_ID_OBC2   11  // from OBC to Broadcast

#define RX_MSG_OBJ_ID_BMS2   15

#define PROCESS_SIZE 8

uint32_t txMsgCount = 0;  // For Slave1/2
uint32_t rxMsgCount = 0;  // For Slave1/2
uint32_t txMsgCount1 = 0; // For Master to Slave1
uint32_t txMsgCount2 = 0; // For Master to Slave2
uint32_t rxMsgCount1 = 0; // For Master to Slave1
uint32_t rxMsgCount2 = 0; // For Master to Slave2
uint32_t errorFlag = 0;

uint16_t txMsgData[8];
uint16_t txMsgData_OBC1[8];
uint16_t txMsgData_OBC2[8];


uint16_t rxMsgData_BMS[8];
uint16_t rxMsgData_VCU[8];

uint16_t RxCycle_BMS[8],RxCycle_BMS_tmp[8];
uint16_t RxCycle_VCU[8],RxCycle_VCU_tmp[8];

uint16_t rxMsgData[8];

uint16_t ErrCodeRevBMS, ErrCodeRevVCU;

uint16_t Can_Decode(uint16_t* ReadAddr, bool Rev_code, uint32_t ID);

uint16_t Can_Encode(uint16_t* DataTransmit, uint32_t format);

uint16_t Can_Decode(uint16_t* ReadAddr, bool Rev_code, uint32_t ID)
{

    uint32_t uldatatmp, uldatatmp1;
    uint16_t datatmp, datatmp1;

    if(Rev_code)
    {
        switch (ID)
        {
            case 0x376:
            OBC_Status.CurrLim = (uint16_t)(*(ReadAddr+6) & 0x03) << 8;
            OBC_Status.CurrLim += (uint16_t)(*(ReadAddr+7) & 0xFF);
            return 0;

            case 0x51F:
                return 0;

            default:
                return 1;
        }
    }
    else
        return 1;
}

uint16_t Can_Encode(uint16_t* DataTransmit, uint32_t format)
{
    switch (format)
    {
        case 0x572:
        DataTransmit[0] = 0;
        DataTransmit[1] = 0;
        DataTransmit[2] = 0;
        DataTransmit[3] = 0;
        DataTransmit[4] = 0;
        DataTransmit[5] = 0;
        DataTransmit[6] = 0;
        DataTransmit[7] = 0;

        DataTransmit[0] = ((OBC_Status.ObcTemp & 0x03FC)>>2);
        DataTransmit[1] = (uint16_t)(OBC_Status.ObcTemp & 0x0003);

        DataTransmit[2] = (OBC_Status.PluginStatus & 0x1)<<7;
        DataTransmit[2] += (OBC_Status.OBCEnabled & 0x1)<<6;
        DataTransmit[2] += (OBC_Status.OBCOTP & 0x1)<<5;
        DataTransmit[2] += (OBC_Status.OBCOOVP & 0x1);

        DataTransmit[3] = (OBC_Status.OBCOUVP & 0x1)<<7;
        DataTransmit[3] += (OBC_Status.OBCIOCP & 0x1)<<5;
        DataTransmit[3] += (OBC_Status.OBCIOVP & 0x1)<<2;
        DataTransmit[3] += (OBC_Status.OBCIUVP & 0x1)<<1;

        DataTransmit[4] = (uint16_t)(OBC_Status.OBCACCurInf & 0x3FC)>>2;
        DataTransmit[5] = ((uint16_t)(OBC_Status.OBCACCurInf & 0x0003)<<6);

        DataTransmit[5] += (uint16_t)(OBC_Status.OBCACVoltInf & 0xFF00)>>8;
        DataTransmit[6] = (uint16_t)(OBC_Status.OBCACVoltInf & 0x00FF);
        DataTransmit[7] = 0x00;

        return 0;

        case 0x3D6:
        DataTransmit[0] = 0;
        DataTransmit[1] = 0;
        DataTransmit[2] = 0;
        DataTransmit[3] = 0;
        DataTransmit[4] = 0;
        DataTransmit[5] = 0;
        DataTransmit[6] = 0;
        DataTransmit[7] = 0;

        DataTransmit[0] = (OBC_Status.OBCReady4Prechrg & 0x1)<<7;
        DataTransmit[0] += (OBC_Status.OBCWakeup4Plugin & 0x1)<<6;
        DataTransmit[0] += (OBC_Status.OBCFaultStatus & 0x03)<<2;

            return 0;

        default:
            return 1;
    }
}
