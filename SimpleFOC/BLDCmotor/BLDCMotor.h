#ifndef BLDCMotor_H
#define BLDCMotor_H

/******************************************************************************/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

//#define M0_Disable   TIM_Cmd(TIM1, DISABLE);        //鍏抽棴M0杈撳嚭

/******************************************************************************/
extern long sensor_direction;
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;//电压传感器校准
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;
extern float current_limit;
/******************************************************************************/
void Motor_init(void);
void Motor_initFOC(float zero_electric_offset, Direction _sensor_direction);
void loopFOC(void);
void move(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/******************************************************************************/

#endif
