#include "sensor.h"
#include "as5600.h"
#include "foc_utils.h"
#include "iic.h"
#include "retarget.h"
/******************************************************************************/
long cpr;
long velocity_calc_timestamp; // 速度计时，用于计算速度
long angle_data_prev;		  // 获取角度用
float angle_prev;			  // 获取速度用
float full_rotation_offset;	  // 角度累加
/******************************************************************************/
void sensor_init(void)
{
    IIC_GPIO_Config();// AS5600
    cpr = AS5600_CPR;
    printf("sensor: AS5600\r\n");

    HAL_Delay(100);//10ms
    angle_data_prev = getRawCount();
    full_rotation_offset = 0;
    velocity_calc_timestamp = 0; // SysTick->VAL;
    HAL_Delay(50);//5ms
    angle_prev = getAngle();
}

/******************************************************************************/
//当前角度——上一次保存下来的角度 offset是圈数+角度
float getAngle(void)
{
    long angle_data, d_angle;

    angle_data = getRawCount(); // 获取编码器的原始值

    // tracking the number of rotations 跟踪旋转次数
    // in order to expand angle range form [0,2PI] to basically infinity 为了将角度范围从[0,2PI]扩展到基本无穷大
    d_angle = angle_data - angle_data_prev;
    // if overflow happened track it as full rotation 如果发生溢出，请将其跟踪为完全旋转
    if (fabs(d_angle) > (0.8f * cpr))
        full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI;
    // save the current angle value for the next steps 保存当前角度值以供后续步骤使用
    // in order to know if overflow happened 以便知道是否发生溢出
    angle_data_prev = angle_data; // 更新上一次的编码器原始值

    if (full_rotation_offset >= (_2PI * 2000)) // 转动圈数过多后浮点数精度下降，电流增加并最终堵转，每隔一定圈数归零一次
    {										   // 这个问题针对电机长时间连续一个方向转动
        full_rotation_offset = 0;			   // 速度模式，高速转动时每次归零会导致电机抖动一次
        angle_prev = angle_prev - _2PI * 2000;
    }
    if (full_rotation_offset <= (-_2PI * 2000))
    {
        full_rotation_offset = 0;
        angle_prev = angle_prev + _2PI * 2000;
    }

    // return the full angle
    // (number of full rotations)*2PI + current sensor angle
    return (full_rotation_offset + (float) angle_data / cpr * _2PI);
}

/******************************************************************************/
// Shaft velocity calculation
float getVelocity(void)
{
    long now_us;
    float Ts, angle_now, vel;

    // calculate sample time
    now_us = SysTick->VAL;
    if (now_us < velocity_calc_timestamp)
        Ts = (float)(velocity_calc_timestamp - now_us) / 21 * 1e-6f;
    else
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp) / 21 * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if (Ts == 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // current angle
    angle_now = getAngle();
    // velocity calculation
    vel = (angle_now - angle_prev) / Ts;

    // save variables for future pass
    angle_prev = angle_now;
    velocity_calc_timestamp = now_us;
    return vel;
}
/******************************************************************************/
