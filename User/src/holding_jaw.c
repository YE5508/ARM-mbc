#include "holding_jaw.h"

#define JAW_OPENFDCANID 0x01010405//DLC=1,Jaw_Open=Rx_Data[0]

bool Jaw_Open;

void Jaw_Init(void)
{
    Jaw_Open=0;

}
void Jaw_Func(void)
{
    SolenoidValve_On(Solenoid_Channel1,0x00|Jaw_Open);
}
void Jaw_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_Data)
{
    if (Rxheader.RxFrameType!=FDCAN_DATA_FRAME||Rxheader.DataLength<1||Rxheader.IdType!=FDCAN_EXTENDED_ID)
    {
        return;
    }

    if(Rxheader.Identifier == JAW_OPENFDCANID && Rxheader.DataLength == 2 && Rx_Data[0] == 'R')//ASCII 52
    {
        Jaw_Open=Rx_Data[1];
    }
}