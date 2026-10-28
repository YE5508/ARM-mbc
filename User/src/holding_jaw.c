#include "holding_jaw.h"

#define JAW_OPENFDCANID 0x01010405//DLC=2, Rx_Data[0]='R', Jaw_Open_now=Rx_Data[1]


Jaw_Switch_t Jaw_Open;
FDCAN_SendQueueType JawTxqueue;



void Jaw_Init(void)
{
    Jaw_Open.Jaw_Open_now=false;
    Jaw_Open.Jaw_Open_last=false;
    JawTxqueue.Canx=FDCAN3;
    JawTxqueue.Front = JawTxqueue.Rear = 0;

}
void Jaw_Func(void)
{
    if (Jaw_Open.Jaw_Open_now == Jaw_Open.Jaw_Open_last)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef Rxheader = {0};
    uint8_t Rxdata[1];
    Rxheader.DataLength=1;
    Rxheader.IdType=FDCAN_STANDARD_ID;
    Rxheader.Identifier=0x423;

    Rxdata[0]=Jaw_Open.Jaw_Open_now|0x00;
    CAN_Enqueue(&JawTxqueue,Rxheader, Rxdata);
    Jaw_Open.Jaw_Open_last=(bool)Rxdata[0];
    

}
void Jaw_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data)
{
    if (Rxheader.RxFrameType!=FDCAN_DATA_FRAME||Rxheader.DataLength<1||Rxheader.IdType!=FDCAN_EXTENDED_ID)
    {
        return;
    }

    if(Rxheader.Identifier == JAW_OPENFDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'R')//ASCII 52
    {
        Jaw_Open.Jaw_Open_now=Rx_Data[1];
    }
}
