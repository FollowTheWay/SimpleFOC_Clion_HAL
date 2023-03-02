/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "iic.h"
#include "sensor.h"
#include "as5600.h"
#include "BLDCMotor.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "FOCMotor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t aRxBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main (void)
{
    /* USER CODE BEGIN 1 */


    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init ();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config ();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init ();
    MX_USART2_UART_Init ();
    /* USER CODE BEGIN 2 */
    RetargetInit (&huart2);
    HAL_Delay (100);
    sensor_init ();
    LPF_init ();
    PID_init ();

    voltage_power_supply = 12;          // V ��Դ��ѹ
    pole_pairs = 10;                    // ���������������ʵ�����ã���Ȼ�����ϵ��⵫��ʧ�ܵĸ���
    voltage_sensor_align = 2;           // V ��ģ������õ�ֵСһ�����0.5-1����̨������õĴ�һ�����2-3
    voltage_limit = 6;                  // V����ҪΪ���Ƶ�������������ֵ��С��12/1.732=6.9
    velocity_limit = 20;                // rad/s �Ƕ�ģʽʱ�������ת�٣�����ģʽ���ٶ�ģʽ��������
    current_limit = 50;                 // A��foc_current��dc_currentģʽ���Ƶ���������Ϊ0���ٶ�ģʽ��λ��ģʽ������
    torque_controller = Type_voltage;   // ��ǰֻ�е�ѹģʽ
    controller = Type_velocity;         // Type_angle; //Type_torque; //Type_velocity
    PID_velocity.P = 0.5;               // 0.5, �ٶȻ�PI������ֻ��P����������ٵ���
    PID_velocity.I = 0.2;               // 0.2
    P_angle.P = 20;                     // λ�û�������ֻ��P������һ�㲻��Ҫ�Ķ�
    PID_velocity.output_ramp = 20;      // �ٶ�����б�ʣ��������Ҫ��������Ϊ0
    LPF_velocity.Tf = 0.0001;           // Tf����Сһ�㣬�������б�����ã��ٶ��л���ƽ�ȣ����û������ģʽ��б�����ƣ�Tf̫С������׶�����������Ϊ0.02��
    target = 0;

    Motor_init ();
    Motor_initFOC (0, CCW); //(0,UNKNOWN);  //(1.1,CW); ��һ���Ȼ��ƫ�ƽǺͷ�������������������أ��Ժ�����������У׼
    printf ("Motor ready.\r\n");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    /***1000 10ms***/
    // HAL_RCC_GetHCLKFreq()/1000 1ms�ж�һ�Σ���HAL_Delay������ʱ��׼Ϊ1ms
    // HAL_RCC_GetHCLKFreq()/100000  10us�ж�һ�Σ���HAL_Delay������ʱ��׼Ϊ10us
    // HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ�Σ���HAL_Delay������ʱ��׼Ϊ1us
    HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000000);  // ���ò�����ϵͳ�δ�ʱ��
    HAL_UART_Receive_IT (&huart2, (uint8_t *) &aRxBuffer, 1);
    while (1)
    {



//      printf ("%d\r\n", getRawCount ());

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
//        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//        HAL_Delay(10000);
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config (void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler ();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler ();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler (void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq ();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
