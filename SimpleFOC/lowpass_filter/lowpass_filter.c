#include "lowpass_filter.h"
#include "stm32f4xx.h"
#include "sensor.h"
/******************************************************************************/
LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;
/******************************************************************************/
void LPF_init(void)
{
	LPF_current_q.Tf=0.05f;    
	LPF_current_q.y_prev=0;
	LPF_current_q.timestamp_prev=0;  //SysTick->VAL;
	
	LPF_current_d.Tf=0.05f;
	LPF_current_d.y_prev=0;
	LPF_current_d.timestamp_prev=0;
	
	LPF_velocity.Tf=0.0001f;   //Tf设置小一点，配合爬升斜率设置PID_velocity.output_ramp，速度切换更平稳；如果没有爬升模式的斜率限制，Tf太小电机容易抖动。
	LPF_velocity.y_prev=0;
	LPF_velocity.timestamp_prev=0;
}
/******************************************************************************/
//利用嘀嗒时钟计时，嘀嗒时钟为8分频，168M/8=21MHz。时钟从0xFFFFFF到0循环倒计时
float LPFoperator(LowPassFilter* LPF,float x)
{
	unsigned long now_us;
	float dt, alpha, y;
	
	now_us = SysTick->VAL;
	if(now_us < LPF->timestamp_prev)dt = (float)(LPF->timestamp_prev - now_us)/21*1e-6f;
	else
		dt = (float)(0xFFFFFF - now_us + LPF->timestamp_prev)/21*1e-6f;
	LPF->timestamp_prev = now_us;
	if(dt > 0.3f)   //时间过长，大概是程序刚启动初始化，直接返回
	{
		LPF->y_prev = x;
		return x;
	}
	
	alpha = LPF->Tf/(LPF->Tf + dt);
	y = alpha*LPF->y_prev + (1.0f - alpha)*x;
	LPF->y_prev = y;
	
	return y;
}
/******************************************************************************/


