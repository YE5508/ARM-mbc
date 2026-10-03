#ifndef HOLDING_JAW_H
#define HOLDING_JAW_H

#include "includes.h"
#include "bsp_solenoid.h"

extern bool Jaw_Open;


void Jaw_Init(void);
void Jaw_Func(void);//放在
void Jaw_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data);


#endif