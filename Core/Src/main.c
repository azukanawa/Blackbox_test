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
/* Includes
 * ------------------------------------------------------------------*/
#include "main.h"

/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "u8g2.h"

/* USER CODE END Includes */

/* Private typedef
 * -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define
 * ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*********************************************************************************************************
 * 常量定义说明
 * AD9833_MCLK    : DDS芯片外部晶振频率 25MHz
 * AD9833_2POW28  : DDS频率寄存器位数 2^28
 * TEST_FREQ      : 测试激励信号频率 1kHz（正弦波）
 * SHORT_THRES    : 阻抗<50Ω 判定为短路
 * OPEN_THRES     : 阻抗>1MΩ 判定为断路
 * PI             : 圆周率，用于电容/相位计算
 * ADC_SAMPLE_NUM : ADC DMA单次采样点数（200点覆盖2个正弦周期，保证精度）
 *********************************************************************************************************/
#define AD9833_MCLK 25000000UL
#define AD9833_2POW28 268435456UL
#define TEST_FREQ 1000.0f
#define SHORT_THRES 50.0f
#define OPEN_THRES 1000000.0f
#define PI 3.1415926535f
#define ADC_SAMPLE_NUM 200
/* USER CODE END PD */

/* Private macro
 * -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables
 * ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes
 * -----------------------------------------------*/
void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_DMA_Init (void);
static void MX_ADC1_Init (void);
static void MX_SPI1_Init (void);
static void MX_I2C1_Init (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code
 * ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main (void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config ();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_DMA_Init ();
  MX_ADC1_Init ();
  MX_SPI1_Init ();
  MX_I2C1_Init ();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config (void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler ();
    }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit) != HAL_OK)
    {
      Error_Handler ();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_ADC1_Init (void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = { 0 };

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init (&hadc1) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_I2C1_Init (void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init (&hi2c1) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_SPI1_Init (void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init (&hspi1) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void
MX_DMA_Init (void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE ();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE ();
  __HAL_RCC_GPIOA_CLK_ENABLE ();
  __HAL_RCC_GPIOB_CLK_ENABLE ();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (DDS_CS_GPIO_Port, DDS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOB,
                     RELAY_RSEL_1K_Pin | RELAY_RSEL_10K_Pin
                         | RELAY_RSEL_100K_Pin | RELAY_PORT_A_Pin
                         | RELAY_PORT_B_Pin | RELAY_PORT_C_Pin,
                     GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (RELAY_PORT_D_GPIO_Port, RELAY_PORT_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_MEAS_BASIC_Pin */
  GPIO_InitStruct.Pin = KEY_MEAS_BASIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init (KEY_MEAS_BASIC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DDS_CS_Pin RELAY_PORT_D_Pin */
  GPIO_InitStruct.Pin = DDS_CS_Pin | RELAY_PORT_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_RSEL_1K_Pin RELAY_RSEL_10K_Pin
     RELAY_RSEL_100K_Pin RELAY_PORT_A_Pin RELAY_PORT_B_Pin RELAY_PORT_C_Pin */
  GPIO_InitStruct.Pin = RELAY_RSEL_1K_Pin | RELAY_RSEL_10K_Pin
                        | RELAY_RSEL_100K_Pin | RELAY_PORT_A_Pin
                        | RELAY_PORT_B_Pin | RELAY_PORT_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief  向AD9833芯片写入16位SPI控制指令
 * @param  data: 待写入的16位控制字/数据字
 * @retval None
 * @note   遵循AD9833 SPI通信时序：先拉低CS，发送数据，再拉高CS
 */
void
AD9833_Write (uint16_t data)
{
  // 拉低片选引脚：启动SPI通信
  HAL_GPIO_WritePin (DDS_CS_GPIO_Port, DDS_CS_Pin, GPIO_PIN_RESET);
  // 发送16位数据：SPI主机传输，超时时间100ms
  HAL_SPI_Transmit (&hspi1, (uint8_t *)&data, 1, 100);
  // 拉高片选引脚：结束SPI通信
  HAL_GPIO_WritePin (DDS_CS_GPIO_Port, DDS_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  初始化AD9833 DDS芯片，输出1kHz正弦波激励信号
 * @param  None
 * @retval None
 */
void
AD9833_Init (void)
{
  AD9833_Write (0x0100); // 软件复位AD9833芯片
  HAL_Delay (2);
  // freq_word：AD9833频率控制字，根据目标频率1kHz计算得到的32位数值
  uint32_t freq_word = (uint64_t)TEST_FREQ * AD9833_2POW28 / AD9833_MCLK;
  AD9833_Write (0x4000 | (freq_word & 0x3FFF));         // 写入频率控制字低14位
  AD9833_Write (0x8000 | ((freq_word >> 14) & 0x3FFF)); // 写入频率控制字高14位
  AD9833_Write (0x2000);                                // 配置为正弦波输出
}

/**
 * @brief  U8G2库I2C字节发送回调函数
 * @param  u8x8: U8G2库核心控制结构体指针，存储显示配置与状态
 * @param  msg: U8G2库消息类型，仅处理数据发送消息
 * @param  arg_int: 待发送的数据字节长度
 * @param  arg_ptr: 待发送数据的缓冲区指针
 * @retval uint8_t: 固定返回1，表示函数执行成功
 * @note   仅实现I2C数据发送功能，满足OLED通信需求
 */
uint8_t
u8x8_byte_stm32_hal_i2c (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                         void *arg_ptr)
{
  uint8_t *data = (uint8_t *)arg_ptr; // 转换数据指针类型

  // 仅处理U8G2数据发送消息：调用HAL库I2C发送函数
  if (msg == U8X8_MSG_BYTE_SEND)
    {
      // OLED I2C地址0x78，左移1位适配HAL库7位地址格式
      HAL_I2C_Master_Transmit (&hi2c1, 0x78 << 1, data, arg_int, 100);
    }
  return 1; // 返回成功标识
}

/**
 * @brief  U8G2库GPIO与延时回调函数
 * @param  u8x8: U8G2库核心控制结构体指针
 * @param  msg: U8G2库消息类型，仅处理延时消息
 * @param  arg_int: 延时时间（单位：毫秒）
 * @param  arg_ptr: 备用参数指针，本函数未使用
 * @retval uint8_t: 固定返回1，表示函数执行成功
 * @note   仅实现毫秒级延时，满足OLED初始化/刷新时序要求
 */
uint8_t
u8x8_gpio_and_delay_stm32 (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                           void *arg_ptr)
{
  // 处理毫秒延时消息：调用HAL库延时函数
  if (msg == U8X8_MSG_DELAY_MILLI)
    {
      HAL_Delay (arg_int);
    }
  return 1; // 返回成功标识
}

/**
 * @brief  初始化SSD1306 OLED显示屏（128x64 I2C接口）
 * @param  无
 * @retval 无
 * @note   绑定硬件驱动函数，初始化屏幕并清空显示缓冲区
 */
void
OLED_Init (void)
{
  // 配置OLED驱动参数：I2C接口、128x64分辨率、绑定自定义驱动函数
  u8g2_Setup_ssd1306_i2c_128x64_noname_f (
      &u8g2, U8G2_R0, u8x8_byte_stm32_hal_i2c, u8x8_gpio_and_delay_stm32);
  u8g2_InitDisplay (&u8g2);                   // 初始化OLED硬件电路
  u8g2_SetPowerSave (&u8g2, 0);               // 关闭省电模式，点亮屏幕
  u8g2_ClearBuffer (&u8g2);                   // 清空显示缓冲区，避免开机乱码
  u8g2_SetFont (&u8g2, u8g2_font_ncenB08_tr); // 设置默认显示字体
}

/**
 * @brief  关闭所有继电器
 * @param  无
 * @retval 无
 * @note   系统初始化/测量完成后调用，复位硬件状态
 */
void
All_Relay_Off (void)
{
  // 关闭量程选择继电器（1k/10k/100k）
  HAL_GPIO_WritePin (RELAY_RSEL_1K_GPIO_Port, RELAY_RSEL_1K_Pin,
                     GPIO_PIN_RESET);
  HAL_GPIO_WritePin (RELAY_RSEL_10K_GPIO_Port, RELAY_RSEL_10K_Pin,
                     GPIO_PIN_RESET);
  HAL_GPIO_WritePin (RELAY_RSEL_100K_GPIO_Port, RELAY_RSEL_100K_Pin,
                     GPIO_PIN_RESET);
  // 关闭端口选择继电器（A/B/C/D）
  HAL_GPIO_WritePin (RELAY_PORT_A_GPIO_Port, RELAY_PORT_A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (RELAY_PORT_B_GPIO_Port, RELAY_PORT_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (RELAY_PORT_C_GPIO_Port, RELAY_PORT_C_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (RELAY_PORT_D_GPIO_Port, RELAY_PORT_D_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  设置采样电阻量程（继电器切换）
 * @param  res: 量程编号 0=1kΩ 1=10kΩ 2=100kΩ
 * @retval 无
 * @note   切换前先关闭所有继电器，防止短路损坏硬件
 */
void
Set_Sampling_Res (uint8_t res)
{
  All_Relay_Off (); // 复位所有继电器：安全切换量程

  // 根据量程编号闭合对应继电器
  switch (res)
    {
    case 0:
      HAL_GPIO_WritePin (RELAY_RSEL_1K_GPIO_Port, RELAY_RSEL_1K_Pin,
                         GPIO_PIN_SET);
      break;
    case 1:
      HAL_GPIO_WritePin (RELAY_RSEL_10K_GPIO_Port, RELAY_RSEL_10K_Pin,
                         GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin (RELAY_RSEL_100K_GPIO_Port, RELAY_RSEL_100K_Pin,
                         GPIO_PIN_SET);
      break;
    default:
      break;
    }
}

/**
 * @brief  控制测试端口继电器通断
 * @param  port: 端口编号 0=A 1=B 2=C 3=D
 * @param  state: 继电器状态 1=闭合(接通) 0=断开(关闭)
 * @retval 无
 * @note   使用数组映射简化GPIO控制代码，提升可读性
 */
void
Set_Port (uint8_t port, uint8_t state)
{
  // 端口GPIO端口数组映射：编号→硬件端口
  GPIO_TypeDef *port_gpio[]
      = { RELAY_PORT_A_GPIO_Port, RELAY_PORT_B_GPIO_Port,
          RELAY_PORT_C_GPIO_Port, RELAY_PORT_D_GPIO_Port };
  // 端口GPIO引脚数组映射：编号→硬件引脚
  uint16_t port_pin[] = { RELAY_PORT_A_Pin, RELAY_PORT_B_Pin, RELAY_PORT_C_Pin,
                          RELAY_PORT_D_Pin };

  // 控制对应端口继电器
  HAL_GPIO_WritePin (port_gpio[port], port_pin[port],
                     state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  微秒级精准延时函数
 * @param  us: 延时时间（单位：微秒）
 * @retval 无
 * @note   基于SysTick定时器实现，弥补HAL库无微秒延时的缺陷
 */
void
HAL_Delay_us (uint32_t us)
{
  // 计算所需SysTick时钟周期数
  uint32_t ticks = us * (SystemCoreClock / 1000000);
  uint32_t start = SysTick->VAL;
  uint32_t elapsed = 0;

  // 循环计数，达到指定延时时间退出
  while (elapsed < ticks)
    {
      uint32_t curr = SysTick->VAL;
      // 处理SysTick计数器溢出情况
      elapsed
          += (curr > start) ? (SysTick->LOAD - curr + start) : (start - curr);
      start = curr;
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler (void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq ();
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
void
assert_failed (uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
