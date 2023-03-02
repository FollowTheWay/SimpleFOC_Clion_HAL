#include "iic.h"
#include "retarget.h"

#define SDA0_IN  {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=0<<(5*2);}//PB5(MOSI)���븡��
#define SDA0_OUT {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=1<<(5*2);}//PB5(MOSI)�����������
#define READ_SDA0  (GPIOB->IDR&(1<<5))

/**
 * @brief  ��ʼ��IO
 * @param  ��
 * @retval ��
 */
void IIC_GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    IIC_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);
    IIC_SDA_SCL(GPIO_PIN_SET);

}

/*
 * @brief IIC��ʼ�ź�
 */
void IIC_Start(void) {
    IIC_SDA(GPIO_PIN_SET);//������SDA������SCL����ֹ���ִ����ź�
    IIC_SCL(GPIO_PIN_SET);
    HAL_Delay(1);//��ʱ��֤SCL�ߵ�ƽʱ��SDA�½���
    IIC_SDA(GPIO_PIN_RESET);
    HAL_Delay(1);
    IIC_SCL(GPIO_PIN_RESET);//���������SCL�ߣ���ô���ݴ�������е�SDA�ĵ�ƽ�任�ͻᱻ��Ϊ����ʼ�ͽ����ź�
}

/*
 * @brief IICֹͣ�ź�
 */
void IIC_Stop(void) {
    IIC_SDA(GPIO_PIN_RESET);//������SDA������SCL����ֹ���ִ����ź�
    IIC_SCL(GPIO_PIN_RESET);
    HAL_Delay(1);//��ʱ��֤SCL�ߵ�ƽʱ��SDA��������
    IIC_SDA(GPIO_PIN_SET);
    IIC_SCL(GPIO_PIN_SET);
    HAL_Delay(1);
    /*����Ͳ�������SCL�ˣ���ΪIICͨѶ�Ѿ�����*/

}

/*
 * @brief IIC�����ȴ�Ӧ�� 1-fail,0-success
 */
unsigned char IIC_Wait_Ack(void) {
    unsigned char ucErrTime = 0;

    SDA0_IN
    IIC_SDA(GPIO_PIN_SET);
    IIC_SCL(GPIO_PIN_SET);
    HAL_Delay(1);
    while (READ_SDA0 != 0) {
        if (++ucErrTime > 250) {
            SDA0_OUT
            IIC_Stop();
            printf("iic failed\r\n");
            return 1;
        }
    }
    SDA0_OUT
    IIC_SCL(GPIO_PIN_RESET);
    return 0;
}

void IIC0_Ack(void) {
    IIC_SCL(GPIO_PIN_RESET);
    IIC_SDA(GPIO_PIN_RESET);
    HAL_Delay(1);
    IIC_SCL(GPIO_PIN_SET);
    HAL_Delay(1);
    IIC_SCL(GPIO_PIN_RESET);
}

void IIC0_NAck(void) {
    IIC_SCL(GPIO_PIN_RESET);
    IIC_SDA(GPIO_PIN_SET);
    HAL_Delay(1);
    IIC_SCL(GPIO_PIN_SET);
    HAL_Delay(1);
    IIC_SCL(GPIO_PIN_RESET);
}

/*
 * @brief IIC���ֽ�
 * @return ���ض�ȡ����һ���ֽ�
 */
uint8_t Read_IIC_Byte(unsigned char ack) {
    unsigned char i, rcv = 0;

    SDA0_IN
    for (i = 0; i < 8; i++) {
        IIC_SCL(GPIO_PIN_RESET);
        HAL_Delay(1);
        IIC_SCL(GPIO_PIN_SET);
        rcv <<= 1;
        if (READ_SDA0 != 0)rcv++;
        HAL_Delay(1);
    }
    SDA0_OUT
    if (!ack)IIC0_NAck();
    else
        IIC0_Ack();
    return rcv;
}


/*
 * @brief IICд�ֽ�
 * @para  ��д����ֽ�����
 */
void IIC_Write_Byte(uint8_t IIC_Byte) {
    unsigned long i;

    IIC_SCL(GPIO_PIN_RESET);
    for (i = 0; i < 8; i++) {
        if ((IIC_Byte & 0x80) != 0)IIC_SDA(GPIO_PIN_SET);
        else
            IIC_SDA(GPIO_PIN_RESET);
        IIC_Byte <<= 1;
        HAL_Delay(1);
        IIC_SCL(GPIO_PIN_SET);
        HAL_Delay(1);
        IIC_SCL(GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

//void check_device(uint8_t addr) {
//    IIC_Start();
//    IIC_Write_Byte(addr | write_bit);
//    IIC_Wait_Ack();
//    printf("��⵽�豸\n");
//}


/*****************************************************************************/
/*********************************************END OF FILE**********************/
