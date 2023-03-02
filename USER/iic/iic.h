#ifndef    IIC_H_
#define    IIC_H_

#include "stm32f4xx.h"

#define device_address  0xA0//7位地址
#define write_bit       0
#define read_bit        1

//引脚定义
/*******************************************************/
#define IIC_SCL_PIN             GPIO_PIN_4
#define IIC_SDA_PIN             GPIO_PIN_5
#define IIC_GPIO_PORT           GPIOB
#define IIC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

/************************************************************/

/* 带参宏，可以像内联函数一样使用 */
#define IIC_SCL(a)            HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SCL_PIN,a)
#define IIC_SDA(a)            HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SDA_PIN,a)
#define IIC_SDA_SCL(a)    HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SCL_PIN|IIC_SDA_PIN,a)
#define IIC_Read_SDA()    HAL_GPIO_ReadPin(IIC_GPIO_PORT,IIC_SDA_PIN)


void IIC_Write_Byte(uint8_t IIC_Byte);

unsigned char IIC_Wait_Ack(void);

void IIC_Start(void);

void IIC_Stop(void);

void IIC_GPIO_Config(void);

uint8_t Read_IIC_Byte(unsigned char ack);

void check_device(uint8_t addr);


#endif
