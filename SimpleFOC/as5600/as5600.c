#include "as5600.h"
#include "iic.h"

/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C
//#define  RAW_Angle_Lo    0x0D
/*****************************************************************************/
uint16_t ReadAS5600(void) {
    unsigned char dh, dl;

    IIC_Start();
    IIC_Write_Byte((AS5600_Address << 1)|0x00);//0
    IIC_Wait_Ack();
    IIC_Write_Byte(RAW_Angle_Hi);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Write_Byte((AS5600_Address << 1)|0x01);//1
    IIC_Wait_Ack();
    dh = Read_IIC_Byte(1);   //1-ack for next byte 确认下一字节
    dl = Read_IIC_Byte(0);   //0-end trans
    IIC_Stop();
    return ((dh << 8) + dl);
}

// 获取编码器的原始值
uint16_t getRawCount(void) {
    uint16_t val;
    val = ReadAS5600() & 0x0FFF;
    return val;
}

