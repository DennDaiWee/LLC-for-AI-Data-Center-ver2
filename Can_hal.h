
#ifndef CAN_HAL_H_
#define CAN_HAL_H_

#define TXCOUNT  100
#define MSG_DATA_LENGTH    8
#define RX_MSG_OBJ_ID_VCU6   2
#define TX_MSG_OBJ_ID_OBC1   10
#define TX_MSG_OBJ_ID_OBC2   11
#define RX_MSG_OBJ_ID_BMS2   15
#define PROCESS_SIZE 8

extern uint32_t txMsgCount;
extern uint32_t rxMsgCount;
extern uint32_t txMsgCount1;
extern uint32_t rxMsgCount1;
extern uint32_t txMsgCount2;
extern uint32_t rxMsgCount2;
extern uint32_t errorFlag;

extern uint16_t txMsgData[8];
extern uint16_t txMsgData_OBC1[8];
extern uint16_t txMsgData_OBC2[8];
extern uint16_t rxMsgData_BMS[8];
extern uint16_t rxMsgData_VCU[8];
extern uint16_t rxMsgData[8];

extern uint16_t RxCycle_BMS[8],RxCycle_BMS_tmp[8];
extern uint16_t RxCycle_VCU[8],RxCycle_VCU_tmp[8];

extern uint16_t ErrCodeRevBMS, ErrCodeRevVCU;

extern uint16_t Can_Decode(uint16_t* ReadAddr, bool Rev_code, uint32_t format);

extern uint16_t Can_Encode(uint16_t* DataTransmit, uint32_t format);

#endif
