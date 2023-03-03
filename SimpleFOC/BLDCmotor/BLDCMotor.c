#include "BLDCMotor.h"
#include "retarget.h"
#include "pid.h"
#include "FOCMotor.h"
#include "sensor.h"
#include "User_Delay.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
/******************************************************************************/
float target;
/******************************************************************************/
long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;
float current_limit;

/******************************************************************************/
int alignSensor (void);

float velocityOpenloop (float target_velocity);

float angleOpenloop (float target_angle);

/******************************************************************************/
void Motor_init (void)
{
    printf ("MOT: Init\r\n");

    //	new_voltage_limit = current_limit * phase_resistance;
    //	voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
    if (voltage_sensor_align > voltage_limit)
        voltage_sensor_align = voltage_limit;

    // current control loop controls voltage 电流控制回路控制电压
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;
    // velocity control loop controls current 速度控制回路控制电流
    // 如果�?电压模式限制电压，�?�果�?电流模式限制电流
    if (torque_controller == Type_voltage)
        PID_velocity.limit = voltage_limit; // 速度模式的电流限
    else
        PID_velocity.limit = current_limit;
    P_angle.limit = velocity_limit; // 角度模式的速度限制

    // printf("MOT: Enable driver.\r\n");
}

/******************************************************************************/
void Motor_initFOC (float zero_electric_offset, Direction _sensor_direction)
{
    // int exit_flag = 1;

    if (zero_electric_offset != 0)
    {
        // abosolute zero offset provided - no need to align 提供了零点偏移量-无需对齐
        zero_electric_angle = zero_electric_offset;
        // set the sensor direction - default CW
        sensor_direction = _sensor_direction;
    }
    alignSensor (); // 检测零点偏移量和极对数

    // shaft_angle update
    angle_prev = getAngle (); // getVelocity(),make sure velocity=0 after power on
    HAL_Delay (5);//50ms
    shaft_velocity = shaftVelocity (); // 必须调用一次，进入主循�?后速度�?0
    HAL_Delay (5);
    shaft_angle = shaftAngle (); // shaft angle
    if (controller == Type_angle)
        target = shaft_angle; // 角度模式，以当前的�?�度为目标�?�度，进入主�?�?后电机静�?

    HAL_Delay (10);
}
/******************************************************************************/
// 检测零点偏移量和极对数 电机方向
int alignSensor (void)
{
    long i;
    float angle;
    float mid_angle, end_angle;
    float moved;

    printf ("MOT: Align sensor.\r\n");

    if (sensor_direction == UNKNOWN) // 传感器方向 没有设置，需要校正
    {
        // find natural direction 找到自然方向
        // move one electrical revolution forward 向前旋转一圈
        for (i = 0; i <= 500; i++)
        {
            angle = _3PI_2 + _2PI * i / 500.0f;
            setPhaseVoltage (voltage_sensor_align, 0, angle);
            HAL_Delay (1);
        }
        mid_angle = getAngle ();

        for (i = 500; i >= 0; i--)
        {
            angle = _3PI_2 + _2PI * i / 500.0f;
            setPhaseVoltage (voltage_sensor_align, 0, angle);
            HAL_Delay (1);
        }
        end_angle = getAngle ();

        setPhaseVoltage (0, 0, 0);
        HAL_Delay (1);

        printf ("mid__angle=%.4f\r\n", mid_angle);
        printf ("end__angle=%.4f\r\n", end_angle);

        moved = fabs (mid_angle - end_angle);
        if ((mid_angle == end_angle) || (moved < 0.01f)) // 相等或者几乎没有动
        {
            printf ("MOT: Failed to notice movement loop222.\r\n");
//			M0_Disable; // 电机检测不正常，关闭驱动
            return 0;
        }
        else if (mid_angle < end_angle)
        {
            printf ("MOT: sensor_direction==CCW\r\n");
            sensor_direction = CCW;
        }
        else
        {
            printf ("MOT: sensor_direction==CW\r\n");
            sensor_direction = CW;
        }

        printf ("MOT: PP check: ");                     // 计算Pole_Pairs 磁极对
        // 0.5 is arbitrary number it can be lower or higher!
        if (fabs (moved * pole_pairs - _2PI) > 0.5f)
        {
            printf ("fail - estimated pp:");            //失败 - 估计 PP：
            pole_pairs = _2PI / moved + 0.5f;           // �?点数�?整形，四舍五�?
            printf ("%d\r\n", pole_pairs);
        }
        else
            printf ("OK!\r\n");
    }
    else
        printf ("MOT: Skip dir calib.\r\n"); // 跳过校准

    if (zero_electric_angle == 0) // 没有设置，需要
    {
        setPhaseVoltage (voltage_sensor_align, 0, _3PI_2); // 计算零点偏移角度
        HAL_Delay (70);
        zero_electric_angle = _normalizeAngle (_electricalAngle (sensor_direction * getAngle (), pole_pairs));
        HAL_Delay (20);
        printf ("MOT: Zero elec. angle:");
        printf ("%.4f\r\n", zero_electric_angle);
        setPhaseVoltage (0, 0, 0);
        HAL_Delay (20);
    }
    else
        printf ("MOT: Skip offset calib.\r\n");

    return 1;
}

/******************************************************************************/
/**
 * @brief 开环FOC
 */
void loopFOC (void)
{
    if (controller == Type_angle_openloop || controller == Type_velocity_openloop)
        return;

    shaft_angle = shaftAngle ();              // shaft angle
    electrical_angle = electricalAngle (); // electrical angle - need shaftAngle to be called first

    switch (torque_controller)
    {
    case Type_voltage: // no need to do anything really
        break;
    case Type_dc_current:break;
    case Type_foc_current:break;
    default:printf ("MOT: no torque control selected!\r\n");
        break;
    }
    // set the phase voltage - FOC heart function :)
    setPhaseVoltage (voltage.q, voltage.d, electrical_angle);
}

/**
 * @brief 根据main中的模式和target传入各项参数
 * @param new_target
 */
void move (float new_target)
{
    shaft_velocity = shaftVelocity ();

    switch (controller)
    {
    case Type_torque:
        if (torque_controller == Type_voltage)
            voltage.q = new_target; // if voltage torque control
        else
            current_sp = new_target; // if current/foc_current torque control
        break;
    case Type_angle:
        // angle set point
        shaft_angle_sp = new_target;
        // calculate velocity set point
        shaft_velocity_sp = PIDoperator (&P_angle, (shaft_angle_sp - shaft_angle));
        // calculate the torque command
        current_sp = PIDoperator (&PID_velocity, (shaft_velocity_sp - shaft_velocity)); // if voltage torque control
        // if torque controlled through voltage
        if (torque_controller == Type_voltage)
        {
            voltage.q = current_sp;
            voltage.d = 0;
        }
        break;
    case Type_velocity:
        // velocity set point
        shaft_velocity_sp = new_target;
        // calculate the torque command
        current_sp = PIDoperator (&PID_velocity,
                                  (shaft_velocity_sp - shaft_velocity)); // if current/foc_current torque control
        // if torque controlled through voltage control
        if (torque_controller == Type_voltage)
        {
            voltage.q = current_sp; // use voltage if phase-resistance not provided
            voltage.d = 0;
        }
        break;
    case Type_velocity_openloop:
        // velocity control in open loop
        shaft_velocity_sp = new_target;
        voltage.q = velocityOpenloop (shaft_velocity_sp); // returns the voltage that is set to the motor
        voltage.d = 0;
        break;
    case Type_angle_openloop:
        // angle control in open loop
        shaft_angle_sp = new_target;
        voltage.q = angleOpenloop (shaft_angle_sp); // returns the voltage that is set to the motor
        voltage.d = 0;
        break;
    }
}
/******************************************************************************/
// 设置相电�?
void setPhaseVoltage (float Uq, float Ud, float angle_el)
{
    float Uout;
    uint32_t sector;
    float T0, T1, T2;
    float Ta, Tb, Tc;

    if (Ud) // only if Ud and Uq set
    {        // _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle (angle_el + atan2 (Uq, Ud));
    }
    else
    { // only Uq available - no need for atan2 and sqrt
        Uout = Uq / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle (angle_el + _PI_2);
    }

    sector = (angle_el / _PI_3) + 1;
    T1 = _SQRT3 * _sin (sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin (angle_el - (sector - 1.0f) * _PI_3) * Uout;
    T0 = 1 - T1 - T2;

    // calculate the duty cycles(times)
    switch (sector)
    {
    case 1:Ta = T1 + T2 + T0 / 2;
        Tb = T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 2:Ta = T1 + T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 3:Ta = T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T2 + T0 / 2;
        break;
    case 4:Ta = T0 / 2;
        Tb = T1 + T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 5:Ta = T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 6:Ta = T1 + T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T0 / 2;
        break;
    default: // possible error state
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

//	TIM_SetCompare1(TIM1, Ta * PWM_Period);
//	TIM_SetCompare2(TIM1, Tb * PWM_Period);
//	TIM_SetCompare3(TIM1, Tc * PWM_Period);
}

/******************************************************************************/
float velocityOpenloop (float target_velocity)
{
    unsigned long now_us;
    float Ts, Uq;

    now_us = SysTick->VAL; //_micros();
    if (now_us < open_loop_timestamp)
        Ts = (float) (open_loop_timestamp - now_us) / 21 * 1e-6f; // 嘀嗒时钟�?�时�?168M时钟8分�?�是21MHz
    else
        Ts = (float) (0xFFFFFF - now_us + open_loop_timestamp) / 21 * 1e-6f;
    open_loop_timestamp = now_us; // save timestamp for next call
    // quick fix for strange cases (micros overflow)
    if (Ts == 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // calculate the necessary angle to achieve target velocity
    shaft_angle = _normalizeAngle (shaft_angle + target_velocity * Ts);

    Uq = voltage_sensor_align;
    // set the maximal allowed voltage (voltage_limit) with the necessary angle
    setPhaseVoltage (Uq, 0, _electricalAngle (shaft_angle, pole_pairs));

    return Uq;
}

/******************************************************************************/
float angleOpenloop (float target_angle)
{
    unsigned long now_us;
    float Ts, Uq;

    now_us = SysTick->VAL; //_micros();
    if (now_us < open_loop_timestamp)
        Ts = (float) (open_loop_timestamp - now_us) / 21 * 1e-6f; // 嘀嗒时钟�?�时�?168M时钟8分�?�是21MHz
    else
        Ts = (float) (0xFFFFFF - now_us + open_loop_timestamp) / 21 * 1e-6f;
    open_loop_timestamp = now_us; // save timestamp for next call
    // quick fix for strange cases (micros overflow)
    if (Ts == 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // calculate the necessary angle to move from current position towards target angle
    // with maximal velocity (velocity_limit)
    if (fabs (target_angle - shaft_angle) > velocity_limit * Ts)
    {
        shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
        // shaft_velocity = velocity_limit;
    }
    else
    {
        shaft_angle = target_angle;
        // shaft_velocity = 0;
    }

    Uq = voltage_sensor_align;
    // set the maximal allowed voltage (voltage_limit) with the necessary angle
    setPhaseVoltage (Uq, 0, _electricalAngle (shaft_angle, pole_pairs));

    return Uq;
}
/******************************************************************************/
unsigned long rcv2_flag;
/******************************************************************************/
/**
 * @brief 拉取指令
 *
 */
void commander_run (void)
{
    if (rcv2_flag == 1)
    {
        switch (RxBuffer[0])
        {
        case 'H':printf ("Hello World!\r\n");
            break;
        case 'T': // T6.28 将字串转换成浮点型数，
            target = atof ((const char *) (RxBuffer + 1));
            printf ("RX=%.2f\r\n", target);
            break;
        case 'P': // P0.5  设置速度环的P参数
            PID_velocity.P = atof ((const char *) (RxBuffer + 1));
            printf ("P=%.2f\r\n", PID_velocity.P);
            break;
        case 'I': // I0.2  设置速度环的I参数
            PID_velocity.I = atof ((const char *) (RxBuffer + 1));
            printf ("I=%.2f\r\n", PID_velocity.I);
            break;
        case 'V': // V  读实时速度
            printf ("Vel=%.2f\r\n", shaft_velocity);
            break;
        case 'A': // A  读绝对角度
            printf ("Ang=%.2f\r\n", shaft_angle);
            break;
        }
        memset (RxBuffer, 0x00, sizeof (RxBuffer)); //清空数组
        rcv2_flag = 0;
    }
}
/******************************************************************************/