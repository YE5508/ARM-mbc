#ifndef HOLDING_JAW_H
#define HOLDING_JAW_H

#include "includes.h"
#include "bsp_solenoid.h"
#include "FD_Canqueue.h"


typedef struct
{
    bool Jaw_Open_now;
    bool Jaw_Open_last;
} Jaw_Switch_t;

extern Jaw_Switch_t Jaw_Open;
extern FDCAN_SendQueueType JawTxqueue;

void Jaw_Init(void);
void Jaw_Func(void);//放在
void Jaw_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data);


#endif
