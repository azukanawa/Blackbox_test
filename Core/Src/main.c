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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal_adc.h"
#include "u8g2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
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

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
u8g2_t u8g2;                           // U8G2显示控制句柄：OLED屏幕操作核心
uint16_t adc_dma_buf[ADC_SAMPLE_NUM];  // ADC DMA采样缓冲区：存储200次原始采样值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// 底层硬件驱动函数
void AD9833_Write(uint16_t data);
void AD9833_Init(void);
void OLED_Init(void);

// 继电器控制函数
void All_Relay_Off(void);
void Set_Sampling_Res(uint8_t res);
void Set_Port(uint8_t port, uint8_t state);

// 信号采集与处理函数
float Get_ADC_RMS_DMA(uint32_t ch);
float Get_Phase_Diff(void);

// 核心测量函数
uint8_t Auto_Switch_Res(void);
void Measure_Impedance(uint8_t port_x, uint8_t port_y, float *z, float *phase, uint8_t *type);

// 测量模式函数
void Basic_Measure(void);
void Extend_Measure(void);

// 自动识别函数
uint8_t Auto_Detect_Mode(void);

// 工具函数
uint8_t Key_Scan(void);
uint8_t u8x8_byte_stm32_hal_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void HAL_Delay_us(uint32_t us);
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
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    AD9833_Init();    // 初始化波形发生器
    OLED_Init();      // 初始化OLED屏幕
    All_Relay_Off();  // 复位所有继电器
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 10, 20, "Black Box Tester");
    u8g2_DrawStr(&u8g2, 0, 40, "Press Key to Start");
    u8g2_SendBuffer(&u8g2);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint8_t key = Key_Scan();  // 扫描按键

        // 按键按下：自动识别模式并测量
        if (key == 1)
        {
            uint8_t mode = Auto_Detect_Mode();
            if (mode == 0)
                Basic_Measure();  // 执行基础测量
            else
                Extend_Measure();  // 执行发挥测量
        }
        HAL_Delay(10);  // 10ms延时：降低CPU负载
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

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
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
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
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DDS_CS_GPIO_Port, DDS_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
                      RELAY_RSEL_1K_Pin | RELAY_RSEL_10K_Pin | RELAY_RSEL_100K_Pin | RELAY_PORT_A_Pin |
                          RELAY_PORT_B_Pin | RELAY_PORT_C_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RELAY_PORT_D_GPIO_Port, RELAY_PORT_D_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : KEY_MEAS_BASIC_Pin */
    GPIO_InitStruct.Pin = KEY_MEAS_BASIC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_MEAS_BASIC_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DDS_CS_Pin RELAY_PORT_D_Pin */
    GPIO_InitStruct.Pin = DDS_CS_Pin | RELAY_PORT_D_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : RELAY_RSEL_1K_Pin RELAY_RSEL_10K_Pin RELAY_RSEL_100K_Pin RELAY_PORT_A_Pin
                             RELAY_PORT_B_Pin RELAY_PORT_C_Pin */
    GPIO_InitStruct.Pin = RELAY_RSEL_1K_Pin | RELAY_RSEL_10K_Pin | RELAY_RSEL_100K_Pin | RELAY_PORT_A_Pin |
                          RELAY_PORT_B_Pin | RELAY_PORT_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  向AD9833芯片写入16位SPI控制指令
 * @param  data: 待写入的16位控制字/数据字
 * @retval 无
 * @note   遵循AD9833 SPI通信时序：先拉低CS，发送数据，再拉高CS
 */
void AD9833_Write(uint16_t data)
{
    // 拉低片选引脚：启动SPI通信（硬件协议强制要求）
    HAL_GPIO_WritePin(DDS_CS_GPIO_Port, DDS_CS_Pin, GPIO_PIN_RESET);
    // 发送16位数据：SPI主机传输，超时时间100ms
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, 100);
    // 拉高片选引脚：结束SPI通信
    HAL_GPIO_WritePin(DDS_CS_GPIO_Port, DDS_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  初始化AD9833 DDS波形发生器
 * @param  无
 * @retval 无
 * @note   配置输出1kHz正弦波，为黑箱测量提供激励信号
 */
void AD9833_Init(void)
{
    AD9833_Write(0x0100);  // 写入复位指令：重置AD9833所有寄存器
    HAL_Delay(2);          // 延时2ms：等待芯片完成复位（硬件时序要求）

    // 计算频率控制字：公式 FREQ_WORD = Fout * 2^28 / MCLK
    // 强制转换uint64_t：避免32位计算溢出
    uint32_t freq_word = (uint64_t)TEST_FREQ * AD9833_2POW28 / AD9833_MCLK;

    AD9833_Write(0x4000 | (freq_word & 0x3FFF));          // 写入频率寄存器低14位
    AD9833_Write(0x8000 | ((freq_word >> 14) & 0x3FFF));  // 写入频率寄存器高14位

    AD9833_Write(0x2000);  // 配置输出模式：正弦波输出，退出复位状态
}

/**
 * @brief  U8G2库I2C字节发送回调函数（STM32 HAL适配）
 * @param  u8x8: U8G2库核心控制结构体指针，存储显示配置与状态
 * @param  msg: U8G2库消息类型，仅处理数据发送消息
 * @param  arg_int: 待发送的数据字节长度
 * @param  arg_ptr: 待发送数据的缓冲区指针
 * @retval uint8_t: 固定返回1，表示函数执行成功
 * @note   仅实现I2C数据发送功能，满足OLED通信需求
 */
uint8_t u8x8_byte_stm32_hal_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0)
            {
                buffer[buf_idx++] = *data;
                data++;
                arg_int--;
            }
            break;
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_SET_DC:
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;  // 传输开始重置索引，消除脏数据
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            // 你原本正常的发送方式，完全保留
            HAL_I2C_Master_Transmit(&hi2c1, 0x3C << 1, buffer, buf_idx, 100);
            buf_idx = 0;  // 传输结束重置索引，双重保险
            break;
        default:
            return 0;
    }
    return 1;
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
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;
        case U8X8_MSG_DELAY_10MICRO:
            HAL_Delay_us(10);
            break;
        case U8X8_MSG_DELAY_100NANO:
            break;
        default:
            break;
    }
    return 1;
}

/**
 * @brief  初始化SSD1306 OLED显示屏（128x64 I2C接口）
 * @param  无
 * @retval 无
 * @note   绑定硬件驱动函数，初始化屏幕并清空显示缓冲区
 */
void OLED_Init(void)
{
    // 1. 绑定驱动（参数顺序保持你正确的写法）
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hal_i2c, u8x8_gpio_and_delay_stm32);

    // 2. 初始化屏幕硬件
    u8g2_InitDisplay(&u8g2);

    // 3. 上电稳定延时
    HAL_Delay(100);

    // 4. 关闭省电模式（点亮屏幕）
    u8g2_SetPowerSave(&u8g2, 0);

    // 5. 清空显存+刷新
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);

    // 6. 设置字体
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
}

/**
 * @brief  关闭所有继电器
 * @param  无
 * @retval 无
 * @note   系统初始化/测量完成后调用，复位硬件状态
 */
void All_Relay_Off(void)
{
    // 关闭量程选择继电器（1k/10k/100k）
    HAL_GPIO_WritePin(RELAY_RSEL_1K_GPIO_Port, RELAY_RSEL_1K_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_RSEL_10K_GPIO_Port, RELAY_RSEL_10K_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_RSEL_100K_GPIO_Port, RELAY_RSEL_100K_Pin, GPIO_PIN_RESET);
    // 关闭端口选择继电器（A/B/C/D）
    HAL_GPIO_WritePin(RELAY_PORT_A_GPIO_Port, RELAY_PORT_A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_PORT_B_GPIO_Port, RELAY_PORT_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_PORT_C_GPIO_Port, RELAY_PORT_C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_PORT_D_GPIO_Port, RELAY_PORT_D_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  设置采样电阻量程
 * @param  res: 量程编号 0=1kΩ 1=10kΩ 2=100kΩ
 * @retval 无
 * @note   切换前先关闭所有继电器，防止短路损坏硬件
 */
void Set_Sampling_Res(uint8_t res)
{
    All_Relay_Off();  // 复位所有继电器：安全切换量程

    // 根据量程编号闭合对应继电器
    switch (res)
    {
        case 0:
            HAL_GPIO_WritePin(RELAY_RSEL_1K_GPIO_Port, RELAY_RSEL_1K_Pin, GPIO_PIN_SET);
            break;
        case 1:
            HAL_GPIO_WritePin(RELAY_RSEL_10K_GPIO_Port, RELAY_RSEL_10K_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(RELAY_RSEL_100K_GPIO_Port, RELAY_RSEL_100K_Pin, GPIO_PIN_SET);
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
void Set_Port(uint8_t port, uint8_t state)
{
    // 端口GPIO端口数组映射：编号→硬件端口
    GPIO_TypeDef *port_gpio[] = {RELAY_PORT_A_GPIO_Port, RELAY_PORT_B_GPIO_Port, RELAY_PORT_C_GPIO_Port,
                                 RELAY_PORT_D_GPIO_Port};
    // 端口GPIO引脚数组映射：编号→硬件引脚
    uint16_t port_pin[] = {RELAY_PORT_A_Pin, RELAY_PORT_B_Pin, RELAY_PORT_C_Pin, RELAY_PORT_D_Pin};

    // 控制对应端口继电器
    HAL_GPIO_WritePin(port_gpio[port], port_pin[port], state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  微秒级精准延时函数
 * @param  us: 延时时间（单位：微秒）
 * @retval 无
 * @note   基于SysTick定时器实现，弥补HAL库无微秒延时的缺陷
 */
void HAL_Delay_us(uint32_t us)
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
        elapsed += (curr > start) ? (SysTick->LOAD - curr + start) : (start - curr);
        start = curr;
    }
}

/**
 * @brief  DMA方式采集ADC电压有效值（RMS）
 * @param  ch: ADC采集通道编号
 * @retval float: 电压有效值（绝对值，单位：V）
 * @note   200次DMA采样+RMS算法，提升交流信号测量精度
 */
float Get_ADC_RMS_DMA(uint32_t ch)
{
    // 定义通道配置结构体
    ADC_ChannelConfTypeDef sConfig = {0};

    // 配置通道参数
    sConfig.Channel = ch;  // 传入通道号
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  // 239.5周期

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 启动DMA采样
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_SAMPLE_NUM);

    HAL_DMA_PollForTransfer(hadc1.DMA_Handle, HAL_DMA_FULL_TRANSFER, 100);

    HAL_ADC_Stop_DMA(&hadc1);

    // RMS计算
    uint64_t sum_sq = 0;
    for (uint16_t i = 0; i < ADC_SAMPLE_NUM; i++)
    {
        sum_sq += (uint64_t)adc_dma_buf[i] * adc_dma_buf[i];
    }

    float rms_adc = sqrtf((float)sum_sq / ADC_SAMPLE_NUM);
    float voltage = (rms_adc * 3.3f / 4095.0f) - 1.65f;
    return fabsf(voltage);
}

/**
 * @brief  自动匹配采样电阻量程
 * @param  无
 * @retval uint8_t: 最终匹配的量程编号 0=1k 1=10k 2=100k
 * @note   根据采样电压自动调整，保证测量在最佳精度区间
 */
uint8_t Auto_Switch_Res(void)
{
    uint8_t res = 1;  // 默认初始量程：10kΩ
    Set_Sampling_Res(res);
    HAL_Delay(5);                                 // 延时5ms：等待继电器机械吸合稳定
    float v_rs = Get_ADC_RMS_DMA(ADC_CHANNEL_0);  // 读取采样电阻电压

    // 电压过小：切换更大量程，避免信号精度丢失
    if (v_rs < 0.2f && res < 2)
    {
        res++;
        Set_Sampling_Res(res);
        HAL_Delay(5);
    }
    // 电压过大：切换更小量程，避免信号饱和失真
    else if (v_rs > 2.5f && res > 0)
    {
        res--;
        Set_Sampling_Res(res);
        HAL_Delay(5);
    }
    return res;
}

/**
 * @brief  测量指定端口的阻抗、相位、元件类型
 * @param  port_x: 测量正端口编号
 * @param  port_y: 测量负端口编号
 * @param  z: 阻抗值输出指针（单位：Ω）
 * @param  phase: 相位差输出指针（单位：度）
 * @param  type: 元件类型输出指针 0=开路 1=短路 2=电阻 3=电容
 * @retval 无
 * @note   核心测量函数，集成量程自动切换、阻抗计算、类型判断
 */
void Measure_Impedance(uint8_t port_x, uint8_t port_y, float *z, float *phase, uint8_t *type)
{
    All_Relay_Off();                  // 复位所有继电器
    Set_Port(port_x, 1);              // 接通正端口
    Set_Port(port_y, 0);              // 接通负端口
    uint8_t res = Auto_Switch_Res();  // 自动匹配最佳量程
    // 量程对应采样电阻值数组
    float r_samp[] = {1000.0f, 10000.0f, 100000.0f};

    // 采集两路电压：采样电阻电压 + 被测元件电压
    float v_rs = Get_ADC_RMS_DMA(ADC_CHANNEL_0);
    float v_z = Get_ADC_RMS_DMA(ADC_CHANNEL_1);
    float current = v_rs / r_samp[res];  // 欧姆定律计算回路电流

    // 电流极小：判定为开路
    if (current < 1e-7f)
    {
        *z = 1e7f;
        *type = 0;
        *phase = 0;
        return;
    }
    *z = v_z / current;  // 计算阻抗：Z = V元件 / I回路

    // 阻抗极小：判定为短路
    if (*z < SHORT_THRES)
    {
        *type = 1;
        *phase = 0;
        return;
    }
    // 阻抗极大：判定为开路
    if (*z > OPEN_THRES)
    {
        *type = 0;
        *phase = 0;
        return;
    }

    // 计算相位差：区分电阻（相位≈0）和电容（相位≠0）
    *phase = Get_Phase_Diff();
    if (fabsf(*phase) < 10.0f)
    {
        *type = 2;  // 纯电阻
    }
    else
    {
        *type = 3;  // 电容
    }
}

/**
 * @brief  计算两路信号的相位差
 * @param  无
 * @retval float: 相位差绝对值（单位：度）
 * @note   通过检测信号过零点时间差计算相位，用于区分阻容元件
 */
float Get_Phase_Diff(void)
{
    uint32_t t1 = 0, t2 = 0;       // 两路信号过零时间戳
    uint16_t val1, val2;           // ADC原始采样值
    uint8_t flag1 = 0, flag2 = 0;  // 过零检测标志位

    // 最多检测1000次：防止程序卡死
    for (uint32_t i = 0; i < 1000; i++)
    {
        // 采集ADC通道0信号
        ADC_ChannelConfTypeDef sConfig1 = {0};
        sConfig1.Channel = ADC_CHANNEL_0;
        sConfig1.Rank = ADC_REGULAR_RANK_1;
        sConfig1.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig1);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        val1 = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        // 采集ADC通道1信号
        ADC_ChannelConfTypeDef sConfig2 = {0};
        sConfig2.Channel = ADC_CHANNEL_1;
        sConfig2.Rank = ADC_REGULAR_RANK_1;
        sConfig2.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig2);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        val2 = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        // 检测通道0过零点：2048对应1.65V直流偏置中点
        if (val1 > 2048 && flag1 == 0)
        {
            t1 = HAL_GetTick();
            flag1 = 1;
        }
        // 检测通道1过零点
        if (val2 > 2048 && flag2 == 0)
        {
            t2 = HAL_GetTick();
            flag2 = 1;
        }
        // 两路均检测到过零：退出循环
        if (flag1 && flag2) break;
        HAL_Delay_us(10);
    }

    int32_t dt = (int32_t)t2 - (int32_t)t1;      // 计算时间差
    float phase = (float)dt * 360.0f / 1000.0f;  // 时间差转换为相位差
    return fabsf(phase);                         // 返回绝对值
}

/**
 * @brief  基础测量模式（双元件：电阻/电容）
 * @param  无
 * @retval 无
 * @note   自动识别拓扑结构，测量并显示双元件参数
 */
void Basic_Measure(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 12, "Basic Mode");
    u8g2_SendBuffer(&u8g2);

    float z1, z2, z3, z4, phase;
    uint8_t type1, type2, type3, type4;
    char buf[32];  // 字符串显示缓冲区

    // 测量所有端口组合，判断黑箱拓扑结构
    Measure_Impedance(0, 2, &z1, &phase, &type1);
    Measure_Impedance(1, 3, &z2, &phase, &type2);
    Measure_Impedance(0, 3, &z3, &phase, &type3);
    Measure_Impedance(1, 2, &z4, &phase, &type4);

    // 判断拓扑：并联/交叉
    uint8_t is_parallel = 0;
    if (type1 != 0 && type2 != 0 && type3 == 0 && type4 == 0)
        is_parallel = 1;
    else if (type3 != 0 && type4 != 0 && type1 == 0 && type2 == 0)
        is_parallel = 0;

    float z_a, z_b;
    uint8_t type_a, type_b;
    float phase_a, phase_b;

    // 根据拓扑结构测量有效元件
    if (is_parallel)
    {
        Measure_Impedance(0, 2, &z_a, &phase_a, &type_a);
        Measure_Impedance(1, 3, &z_b, &phase_b, &type_b);
    }
    else
    {
        Measure_Impedance(0, 3, &z_a, &phase_a, &type_a);
        Measure_Impedance(1, 2, &z_b, &phase_b, &type_b);
    }

    // 计算元件参数：电阻/电容/品质因数
    float r_a = 0, c_a = 0, q_a = 0;
    float r_b = 0, c_b = 0, q_b = 0;
    if (type_a == 2)
    {
        r_a = z_a;
        q_a = 999.0f;
    }  // 电阻Q值标记为999
    else if (type_a == 3)
    {
        c_a = 1.0f / (2 * PI * TEST_FREQ * z_a) * 1e9f;
        q_a = tanf(phase_a * PI / 180);
    }
    if (type_b == 2)
    {
        r_b = z_b;
        q_b = 999.0f;
    }
    else if (type_b == 3)
    {
        c_b = 1.0f / (2 * PI * TEST_FREQ * z_b) * 1e9f;
        q_b = tanf(phase_b * PI / 180);
    }

    // OLED显示测量结果
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 10, is_parallel ? "Topo: Parallel" : "Topo: Cross");
    u8g2_DrawStr(&u8g2, 0, 22, "Ch1:");
    if (type_a == 0)
        u8g2_DrawStr(&u8g2, 30, 22, "Open");
    else if (type_a == 1)
        u8g2_DrawStr(&u8g2, 30, 22, "Short");
    else if (type_a == 2)
    {
        sprintf(buf, "R=%.1f Ohm", r_a);
        u8g2_DrawStr(&u8g2, 30, 22, buf);
    }
    else if (type_a == 3)
    {
        sprintf(buf, "C=%.1f nF", c_a);
        u8g2_DrawStr(&u8g2, 30, 22, buf);
        sprintf(buf, "Q=%.1f", q_a);
        u8g2_DrawStr(&u8g2, 30, 32, buf);
    }

    u8g2_DrawStr(&u8g2, 0, 44, "Ch2:");
    if (type_b == 0)
        u8g2_DrawStr(&u8g2, 30, 44, "Open");
    else if (type_b == 1)
        u8g2_DrawStr(&u8g2, 30, 44, "Short");
    else if (type_b == 2)
    {
        sprintf(buf, "R=%.1f Ohm", r_b);
        u8g2_DrawStr(&u8g2, 30, 44, buf);
    }
    else if (type_b == 3)
    {
        sprintf(buf, "C=%.1f nF", c_b);
        u8g2_DrawStr(&u8g2, 30, 44, buf);
        sprintf(buf, "Q=%.1f", q_b);
        u8g2_DrawStr(&u8g2, 30, 54, buf);
    }

    u8g2_SendBuffer(&u8g2);
    All_Relay_Off();  // 测量完成，复位继电器
}

/**
 * @brief  拓展测量模式（三元件+短路检测）
 * @param  无
 * @retval 无
 * @note   测量三元件参数，自动识别短路位置，完整显示Q值
 */
void Extend_Measure(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 12, "Extend Mode");
    u8g2_SendBuffer(&u8g2);

    float z1, z2, z3, phase;
    uint8_t type1, type2, type3;
    char buf[32];

    // 测量拓展题核心端口
    Measure_Impedance(0, 2, &z1, &phase, &type1);
    Measure_Impedance(1, 3, &z2, &phase, &type2);
    Measure_Impedance(0, 1, &z3, &phase, &type3);

    // 判断题型：I型/II型（含短路）
    uint8_t is_type1 = 0;
    uint8_t short_pos = 0;
    if (type1 != 1 && type2 != 1)
        is_type1 = 1;
    else
    {
        is_type1 = 0;
        short_pos = (type1 == 1) ? 1 : 2;
    }

    // 计算Unk3元件参数（公共端元件）
    float r3 = 0, c3 = 0, q3 = 0;
    if (type3 == 2)
    {
        r3 = z3;
        q3 = 999.0f;
    }
    else if (type3 == 3)
    {
        c3 = 1.0f / (2 * PI * TEST_FREQ * z3) * 1e9f;
        q3 = tanf(phase * PI / 180);
    }

    // 计算Unk1元件参数
    float r1 = 0, c1 = 0, q1 = 0;
    if (type1 == 2)
    {
        r1 = z1;
        q1 = 999.0f;
    }
    else if (type1 == 3)
    {
        c1 = 1.0f / (2 * PI * TEST_FREQ * z1) * 1e9f;
        q1 = tanf(phase * PI / 180);
    }

    // 计算Unk2元件参数
    float r2 = 0, c2 = 0, q2 = 0;
    if (type2 == 2)
    {
        r2 = z2;
        q2 = 999.0f;
    }
    else if (type2 == 3)
    {
        c2 = 1.0f / (2 * PI * TEST_FREQ * z2) * 1e9f;
        q2 = tanf(phase * PI / 180);
    }

    // OLED显示测量结果
    u8g2_ClearBuffer(&u8g2);
    // 显示题型
    u8g2_DrawStr(&u8g2, 0, 10, is_type1 ? "Type: I" : "Type: II");
    if (!is_type1)
    {
        sprintf(buf, "Short: Unk%d", short_pos);
        u8g2_DrawStr(&u8g2, 60, 10, buf);
    }

    // 显示Unk3
    u8g2_DrawStr(&u8g2, 0, 22, "Unk3:");
    if (type3 == 2)
    {
        sprintf(buf, "R=%.1f Ohm", r3);
        u8g2_DrawStr(&u8g2, 35, 22, buf);
    }
    else if (type3 == 3)
    {
        sprintf(buf, "C=%.1f nF", c3);
        u8g2_DrawStr(&u8g2, 35, 22, buf);
        sprintf(buf, "Q=%.1f", q3);
        u8g2_DrawStr(&u8g2, 35, 32, buf);
    }

    // 显示Unk1
    u8g2_DrawStr(&u8g2, 0, 44, "Unk1:");
    if (type1 == 1)
        u8g2_DrawStr(&u8g2, 35, 44, "Short");
    else if (type1 == 2)
    {
        sprintf(buf, "R=%.1f Ohm", r1);
        u8g2_DrawStr(&u8g2, 35, 44, buf);
    }
    else if (type1 == 3)
    {
        sprintf(buf, "C=%.1f nF", c1);
        u8g2_DrawStr(&u8g2, 35, 44, buf);
        sprintf(buf, "Q=%.1f", q1);
        u8g2_DrawStr(&u8g2, 35, 54, buf);
    }

    // 显示Unk2
    u8g2_DrawStr(&u8g2, 64, 44, "Unk2:");
    if (type2 == 1)
        u8g2_DrawStr(&u8g2, 99, 44, "Short");
    else if (type2 == 2)
    {
        sprintf(buf, "R=%.1f", r2);
        u8g2_DrawStr(&u8g2, 99, 44, buf);
    }
    else if (type2 == 3)
    {
        sprintf(buf, "C=%.1f", c2);
        u8g2_DrawStr(&u8g2, 99, 44, buf);
        sprintf(buf, "Q=%.1f", q2);
        u8g2_DrawStr(&u8g2, 99, 54, buf);
    }

    u8g2_SendBuffer(&u8g2);
    All_Relay_Off();
}

/**
 * @brief  自动识别测量模式
 * @param  无
 * @retval uint8_t: 0=基础模式 1=发挥模式
 * @note   通过测量0-1端口判断题型：开路=基础题，有元件=发挥题
 */
uint8_t Auto_Detect_Mode(void)
{
    float z, phase;
    uint8_t type;

    // 核心判断逻辑：测量A-B端口
    Measure_Impedance(0, 1, &z, &phase, &type);

    // 端口有元件：发挥模式
    if (type != 0) return 1;
    // 端口开路：基础模式
    else
        return 0;
}

/**
 * @brief  单按键扫描函数（一键启动测量）
 * @param  无
 * @retval uint8_t: 1=有效按键按下 0=无按键
 * @note   集成软件消抖+自锁机制，防止长按重复触发
 */
uint8_t Key_Scan(void)
{
    static uint8_t key_lock = 0;  // 静态自锁标志：仅初始化一次
    uint8_t key = 0;              // 按键返回值

    // 检测按键电平：高电平为按下（硬件上拉输入）
    if (HAL_GPIO_ReadPin(KEY_MEAS_BASIC_GPIO_Port, KEY_MEAS_BASIC_Pin) == GPIO_PIN_SET)
    {
        HAL_Delay(10);  // 10ms软件消抖：滤除机械抖动干扰
        // 二次确认按下 + 未自锁：判定为有效按键
        if (HAL_GPIO_ReadPin(KEY_MEAS_BASIC_GPIO_Port, KEY_MEAS_BASIC_Pin) == GPIO_PIN_SET && !key_lock)
        {
            key_lock = 1;  // 自锁：松开前不再响应
            key = 1;       // 返回有效按键
        }
    }
    else
    {
        key_lock = 0;  // 按键松开：解除自锁
    }
    return key;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
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
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
