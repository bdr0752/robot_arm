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
#include "ws2812.h"

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
/* 保存通信结果，供调试器观察。 */
static volatile FtServoResult r_ping;
static volatile FtServoResult r_model;
static volatile FtServoResult r_mode;
static volatile FtServoResult r_position;
static volatile FtServoResult r_torque;
static volatile FtServoResult r_min;
static volatile FtServoResult r_max;

static volatile FtServoResult min_angle_result;
static volatile FtServoResult max_angle_result;

/* 调试器中改为 1：执行一次保持位置测试。 */
static volatile uint8_t hold_request = 0U;
static volatile uint8_t hold_step = 0U;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* 板级初始化内部依次配置 GPIO、DMA、USART3，并绑定舵机串口管理对象。 */
  Bsp_Board_Init();
  ws2812_init();
  HAL_Delay(100);
  /* 与刚刚 PING 成功时使用的 ID 保持一致。 */
  const uint8_t servo_id = 0x06U;
  
  uint8_t status = 0;
  uint8_t mode = 1;
  uint16_t model = 0;
  uint16_t position = 0;
  
  uint8_t torque_enabled = 0xFFU;
  uint16_t min_position = 0U;
  uint16_t max_position = 0U;
  
  uint8_t torque_status = 0xFFU;
  uint8_t min_status = 0xFFU;
  uint8_t max_status = 0xFFU;
  
  /* 读取扭矩使能状态，不改变它。 */
  r_torque = ft_servo_read_byte(
  	servo_id, FT_SMS_REG_TORQUE_ENABLE,
  	&torque_enabled, &torque_status);
  
  /* 读取舵机配置的位置下限。 */
  r_min = ft_servo_read_word(
  	servo_id, FT_SMS_REG_MIN_ANGLE_L,
  	&min_position, &min_status);
  
  /* 读取舵机配置的位置上限。 */
  r_max = ft_servo_read_word(
		servo_id, FT_SMS_REG_MAX_ANGLE_L,
		&max_position, &max_status);

  


  r_ping = ft_servo_ping(servo_id, &status);
  r_model = ft_servo_read_word(
      servo_id, FT_SMS_REG_MODEL_L, &model, &status);
  r_mode = ft_servo_set_mode(servo_id, FT_SMS_MODE_WHEEL, &status);
  r_position = ft_servo_read_position(
      servo_id, &position, &status);
	  
  /* 测试结果，供调试器观察。 */

//  uint16_t hold_target = 0U;
//  uint16_t hold_goal_readback = 0U;
//  uint16_t hold_feedback = 0U;
//  uint8_t hold_mode = 0xFFU;
//  uint8_t hold_torque = 0xFFU;
//  uint8_t hold_status = 0xFFU;
//  FtServoResult hold_result = FT_SERVO_OK;
  
  
  Ft_SetPosition(servo_id,2048,5,5);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// 	  if (hold_request == 1U) {
//     /* 清除请求，本次只执行一次。 */
//     hold_request = 0U;

//     do {
//         /* 第一步：确认仍是位置模式。 */
//         hold_step = 1U;
//         hold_result = ft_servo_read_mode(
//             servo_id, &hold_mode, &hold_status);
//         if (hold_result != FT_SERVO_OK ||
//             hold_status != 0U ||
//             hold_mode != FT_SMS_MODE_POSITION) {
//             break;
//         }

//         /* 第二步：读取最新位置作为目标。 */
//         hold_step = 2U;
//         hold_result = ft_servo_read_position(
//             servo_id, &hold_target, &hold_status);
//         if (hold_result != FT_SERVO_OK || hold_status != 0U) {
//             break;
//         }

//         /* 确认此前读取的上下限有效，且当前位置在其中。 */
//         if (r_min != FT_SERVO_OK || r_max != FT_SERVO_OK ||
//             min_status != 0U || max_status != 0U ||
//             min_position > max_position ||
//             hold_target < min_position ||
//             hold_target > max_position) {
//             break;
//         }

//         /* 第三步：只写目标位置寄存器。 */
//         hold_step = 3U;
//         hold_result = ft_servo_write_word(
//             servo_id, FT_SMS_REG_GOAL_POS_L,
//             hold_target, &hold_status);
//         if (hold_result != FT_SERVO_OK || hold_status != 0U) {
//             break;
//         }

//         /* 第四步：读回目标，确认写入生效。 */
//         hold_step = 4U;
//         hold_result = ft_servo_read_word(
//             servo_id, FT_SMS_REG_GOAL_POS_L,
//             &hold_goal_readback, &hold_status);
//         if (hold_result != FT_SERVO_OK ||
//             hold_status != 0U ||
//             hold_goal_readback != hold_target) {
//             break;
//         }

//         /* 第五步：开启扭矩，保持刚设置的位置。 */
//         hold_step = 5U;
//         hold_result = ft_servo_set_torque(
//             servo_id, 1U, &hold_status);
//         if (hold_result != FT_SERVO_OK || hold_status != 0U) {
//             break;
//         }

//         /* 第六步：确认扭矩使能值为 1。 */
//         hold_step = 6U;
//         hold_result = ft_servo_read_byte(
//             servo_id, FT_SMS_REG_TORQUE_ENABLE,
//             &hold_torque, &hold_status);
//         if (hold_result != FT_SERVO_OK ||
//             hold_status != 0U || hold_torque != 1U) {
//             break;
//         }

//         /* 第七步：等待后采样位置反馈。 */
//         hold_step = 7U;
//         HAL_Delay(500);
//         hold_result = ft_servo_read_position(
//             servo_id, &hold_feedback, &hold_status);
//         if (hold_result != FT_SERVO_OK || hold_status != 0U) {
//             break;
//         }

//         hold_step = 8U;  /* 本轮测试完成。 */
//     } while (0);
// }
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
