/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_board.h"
#include "ft_Servo.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "vofa.h"
#include "ws2812.h"
#include "vofa.h"
#include <stdint.h>
#include "servo_speed_loop.h"
#include "bsp_uart.h"
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static volatile FtServoResult r_mode;
// static volatile FtServoResult r_position;
static volatile FtServoResult r_torque;



const Struct_PID_Config *config = &(Struct_PID_Config){
    .K_P = 0.0f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .Out_Min = -1000.0f,
    .Out_Max = 1000.0f,
    .I_Out_Min = -500.0f,
    .I_Out_Max = 500.0f,
    .D_Filter_Enable = false,
    .D_Filter_Time = 0.0f
};

ServoSpeedLoop *loop = &(ServoSpeedLoop){
    .id = 0x03U,
    .pid = (Struct_PID){0},
    .target_speed = 0.0f,
    .measured_speed = 0,
    .pwm = 0
};

float Channel[5] = {0};
static volatile float target_speed = 0.0f;
uint32_t time_now = 0.0f;
uint32_t time_last = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


///**
//  * @brief  解析命令，直接修改对应参数。
//  * @param  Line: 不含换行、可修改的命令字符串。
//  * @retval 无；无效命令忽略。
//  */
//static void Vofa_HandleCommand(char *Line)
//{
//    char *Text = strchr(Line, '=');
//    if (Text == NULL) {
//        return;
//    }
//    *Text++ = '\0';

//    volatile float *Parameter;

//    if (strcmp(Line, "kp") == 0) {
//        Parameter = &loop->pid.Config.K_P;
//    } else if (strcmp(Line, "ki") == 0) {
//        Parameter = &loop->pid.Config.K_I;
//    } else if (strcmp(Line, "kd") == 0) {
//        Parameter = &loop->pid.Config.K_D;
//    } else if (strcmp(Line, "speed") == 0) {
//        Parameter = &target_speed;
//    } else {
//        return;
//    }

//    float Value = 0.0f;
//    float Scale = 1.0f;
//    bool Decimal = false;
//    bool Has_Digit = false;
//    bool Negative = (*Text == '-');

//    if (Negative) {
//        Text++;
//    }

//    for (; *Text != '\0'; Text++) {
//        if (*Text == '.' && !Decimal) {
//            Decimal = true;
//        } else if (*Text >= '0' && *Text <= '9') {
//            Has_Digit = true;
//            if (Decimal) {
//                Scale *= 0.1f;
//                Value += (float)(*Text - '0') * Scale;
//            } else {
//                Value = Value * 10.0f + (float)(*Text - '0');
//            }
//        } else {
//            return;
//        }
//    }

//    Value = Negative ? -Value : Value;

//    if (!Has_Digit || !isfinite(Value)) {
//        return;
//    }
//    if (Parameter == &target_speed) {
//        if (fabsf(Value) > 32767.0f) {
//            return;
//        }
//    } else if (Value < 0.0f) {
//        return;
//    }

//    *Parameter = Value;
//}

///**
//  * @brief  拼接 UART 数据，收到换行后处理命令。
//  * @param  Manager: 接收管理对象。
//  * @param  Data: 本次收到的数据。
//  * @param  Size: 数据字节数。
//  * @retval 无。
//  */
//static void Vofa_RxCallback(BspUartManager *Manager,
//                            const uint8_t *Data, uint16_t Size)
//{
//    static char Line[VOFA_RX_LINE_SIZE];
//    static uint16_t Length = 0U;

//    (void)Manager;

//    for (uint16_t i = 0U; i < Size; i++) {
//        char Byte = (char)Data[i];

//        if (Byte == '\r') {
//            continue;
//        }

//        if (Byte == '\n') {
//            if (Length > 0U && Length < sizeof(Line)) {
//                Line[Length] = '\0';
//                Vofa_HandleCommand(Line);
//            }
//            Length = 0U;
//        } else if (Byte != '\0' && Length < sizeof(Line) - 1U) {
//            Line[Length++] = Byte;
//        } else {
//            /* 超长或含 NUL 的命令忽略到下一次换行。 */
//            Length = sizeof(Line);
//        }
//    }
//    Bsp_Uart_StartReceiveToIdle(Manager);
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  /* 板级初始化内部配置 GPIO、DMA、USART3、USART1，并绑定串口管理对象。 */
  Bsp_Board_Init();

//  BspUartManager *Debug_Uart = Bsp_Board_DebugUart();
//  Bsp_Uart_SetRxCallback(Debug_Uart, Vofa_RxCallback);
  Vofa_Init();

  ServoSpeedLoop_Init(loop, 0x03U, config);

  ws2812_init();
  ws2812_set_pixel(0U, (Ws2812Color){.red = 255U, .green = 0U, .blue = 0U});
  ws2812_show();

  HAL_Delay(100);

  uint8_t status = 0;
  
  uint8_t torque_status = 0xFFU;
  uint8_t pid_status = 0xFFU;
  float D_T = 0.0f;
  
  /* 关闭扭矩 */
  r_torque = ft_servo_set_torque(loop->id, 0U, &torque_status);
  /* 设置 PWM 模式 */
  r_mode = ft_servo_set_mode(loop->id, FT_SMS_MODE_PWM, &status);
  /* 开启扭矩，开始输出 */
  r_torque = ft_servo_set_torque(loop->id, 1U, &torque_status);
  time_last = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Channel[0] = (float)loop->measured_speed;
    Channel[1] = (float)loop->pid.Config.K_P;
    Channel[2] = (float)loop->pid.Config.K_I;
    Channel[3] = (float)loop->pid.Config.K_D;
    Channel[4] = target_speed;
    time_now = HAL_GetTick();
    D_T = (float)(time_now - time_last);
    if(D_T >= 10.0f){
      time_last = time_now;
      ServoSpeedLoop_Update(loop, target_speed, D_T / 1000.0f, &pid_status);
    }
//    Vofa_Send(Channel, 5U);
    
    
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
