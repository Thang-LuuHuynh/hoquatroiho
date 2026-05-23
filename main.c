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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI              3.14159265f
#define WHEEL_DIAMETER  0.063f       /* Duong kinh banh (m) */
#define WHEEL_BASE      0.17f        /* Khoang cach 2 banh (m) */
#define PPR             937.0f       /* Encoder pulses/revolution */
#define MAX_RPM         150.0f
#define TS_PID          0.01f        /* PID loop: 100Hz = 10ms */
#define TS_SENSOR       0.05f        /* Sensor/comm loop: 20Hz = 50ms */

/* MPU9250 (Gyro + Accel) */
#define MPU9250_ADDR    0xD0
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT1   0x6B
#define REG_GYRO_CFG    0x1B
#define REG_GYRO_ZOUT   0x47
#define REG_INT_PIN_CFG 0x37

/* AK8963 (Magnetometer) inside MPU9250 */
#define AK8963_ADDR     0x18         /* 0x0C << 1 */
#define AK8963_WIA      0x00
#define AK8963_CNTL1    0x0A
#define AK8963_ST1      0x02
#define AK8963_HXL      0x03
#define AK8963_ST2      0x09

/* Complementary filter coefficient */
#define CF_ALPHA        0.98f         /* Higher = trust gyro more */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// PID
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float last_error;
    float output;
} PID_TypeDef;

PID_TypeDef pidL = {2.48f, 24.10f, 0.03f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_TypeDef pidR = {2.44f, 24.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// ENCODER
int32_t last_countL = 0, last_countR = 0;
float   rpmL = 0.0f, rpmR = 0.0f;
float   rpmL_signed = 0.0f, rpmR_signed = 0.0f;
int8_t  dirL = 1, dirR = 1;

// IMU - Gyro
float   gyro_z_rads   = 0.0f;
float   gyro_z_offset = 0.0f;
uint8_t mpu_ok        = 0;

// Magnetometer
uint8_t mag_ok      = 0;
float   mag_bias_x  = 0.0f;
float   mag_bias_y  = 0.0f;
float   fused_yaw   = 0.0f;   /* rad, complementary filter output */

// UART
uint8_t rx_buffer[64];
char    uart_buf[64];

// Loop counter for decoupling
static uint8_t loop_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  HAL_TIM_Base_Start_IT(&htim10);
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5, 0);

  // ── Init MPU9250 (Gyro) ─────────────────────────────────────────────────
  {
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, REG_WHO_AM_I, 1, &who, 1, 100) == HAL_OK
        && who == 0x71)
    {
      // Wake up MPU9250
      uint8_t val = 0x00;
      HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, REG_PWR_MGMT1, 1, &val, 1, 100);
      HAL_Delay(10);

      // Gyro full scale ±250 deg/s
      val = 0x00;
      HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, REG_GYRO_CFG, 1, &val, 1, 100);
      HAL_Delay(10);

      // Enable I2C bypass for AK8963 (magnetometer)
      val = 0x02;
      HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, REG_INT_PIN_CFG, 1, &val, 1, 100);
      HAL_Delay(10);

      // Calc gyro zero offset
      int32_t sum = 0;
      uint8_t raw[2];
      for (int i = 0; i < 100; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, REG_GYRO_ZOUT, 1, raw, 2, 100) == HAL_OK)
          sum += (int16_t)((raw[0] << 8) | raw[1]);
        HAL_Delay(5);
      }
      gyro_z_offset = (float)sum / 100.0f;
      mpu_ok = 1;
      HAL_UART_Transmit(&huart2, (uint8_t*)"GYRO OK\r\n", 10, 100);
    }
    else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"MPU FAIL\r\n", 10, 100);
    }
  }

  // ── Init AK8963 (Magnetometer) ──────────────────────────────────────────
  {
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_WIA, 1, &who, 1, 100) == HAL_OK
        && who == 0x48)
    {
      // Reset AK8963
      uint8_t val = 0x00;
      HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, AK8963_CNTL1, 1, &val, 1, 100);
      HAL_Delay(10);

      // Continuous measurement mode 2: 100Hz, 16-bit
      val = 0x16;
      HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, AK8963_CNTL1, 1, &val, 1, 100);
      HAL_Delay(10);

      // ── Hard-iron calibration ──────────────────────────────────────────
      int16_t mx_min = 32767, mx_max = -32768;
      int16_t my_min = 32767, my_max = -32768;
      uint8_t raw[6];

      HAL_UART_Transmit(&huart2, (uint8_t*)"MAG CALIB START\r\n", 18, 100);
      HAL_UART_Transmit(&huart2, (uint8_t*)"  Quay robot cham 1 vong...\r\n", 30, 100);

      for (int i = 0; i < 100; i++) {
        uint8_t st1;
        if (HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_ST1, 1, &st1, 1, 10) == HAL_OK
            && (st1 & 0x01))
        {
          HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_HXL, 1, raw, 6, 10);
          HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_ST2, 1, &st1, 1, 10);

          int16_t mx = (int16_t)((raw[1] << 8) | raw[0]);
          int16_t my = (int16_t)((raw[3] << 8) | raw[2]);

          if (mx < mx_min) mx_min = mx;
          if (mx > mx_max) mx_max = mx;
          if (my < my_min) my_min = my;
          if (my > my_max) my_max = my;
        }
        HAL_Delay(20);
      }

      mag_bias_x = (float)(mx_max + mx_min) / 2.0f;
      mag_bias_y = (float)(my_max + my_min) / 2.0f;

      char buf[48];
      int len = snprintf(buf, sizeof(buf),
          "MAG BIAS: %.0f %.0f\r\n", mag_bias_x, mag_bias_y);
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);

      mag_ok = 1;
      HAL_UART_Transmit(&huart2, (uint8_t*)"MAG OK\r\n", 8, 100);
    }
    else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"MAG FAIL\r\n", 10, 100);
    }
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)"SYSTEM READY\r\n", 14, 100);

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
void SystemClock_Config(void)
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM10) return;

  /* ── PID LOOP: 100Hz (every 10ms) ── */
  int32_t cntL = -(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  int32_t cntR =  (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

  float raw_rpmL = (float)(cntL - last_countL) * 60.0f / (PPR * TS_PID);
  float raw_rpmR = (float)(cntR - last_countR) * 60.0f / (PPR * TS_PID);
  last_countL = cntL;
  last_countR = cntR;

  rpmL = (raw_rpmL < 0 ? -raw_rpmL : raw_rpmL) * 1.22f;
  rpmR = (raw_rpmR < 0 ? -raw_rpmR : raw_rpmR);

  rpmL_signed = raw_rpmL * 1.22f;
  rpmR_signed = raw_rpmR;

  /* ── PID Left ── */
  {
    float err = pidL.setpoint - rpmL;
    float prop = pidL.Kp * err;
    float deriv = pidL.Kd * (err - pidL.last_error) / TS_PID;

    pidL.output = prop + pidL.Ki * pidL.integral + deriv;

    if (pidL.output > 999.0f) {
      pidL.output = 999.0f;
      if (err < 0) pidL.integral += err * TS_PID;
    } else if (pidL.output < 0.0f) {
      pidL.output = 0.0f;
      if (err > 0) pidL.integral += err * TS_PID;
    } else {
      pidL.integral += err * TS_PID;
    }

    pidL.last_error = err;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pidL.output);
  }

  /* ── PID Right ── */
  {
    float err = pidR.setpoint - rpmR;
    float prop = pidR.Kp * err;
    float deriv = pidR.Kd * (err - pidR.last_error) / TS_PID;

    pidR.output = prop + pidR.Ki * pidR.integral + deriv;

    if (pidR.output > 999.0f) {
      pidR.output = 999.0f;
      if (err < 0) pidR.integral += err * TS_PID;
    } else if (pidR.output < 0.0f) {
      pidR.output = 0.0f;
      if (err > 0) pidR.integral += err * TS_PID;
    } else {
      pidR.integral += err * TS_PID;
    }

    pidR.last_error = err;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pidR.output);
  }

  /* ── SENSOR + COMM LOOP: 20Hz (every 5th call) ── */
  loop_cnt++;
  if (loop_cnt >= 5) {
    loop_cnt = 0;

    // Read Gyro Z
    if (mpu_ok) {
      uint8_t raw[2];
      if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, REG_GYRO_ZOUT, 1, raw, 2, 10) == HAL_OK) {
        int16_t z = (int16_t)((raw[0] << 8) | raw[1]);
        gyro_z_rads = -((float)z - gyro_z_offset) / 131.0f * (PI / 180.0f);
      }
    }

    // Read Magnetometer + Complementary Filter
    if (mag_ok) {
      uint8_t st1;
      uint8_t raw[6];
      if (HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_ST1, 1, &st1, 1, 10) == HAL_OK
          && (st1 & 0x01))
      {
        HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_HXL, 1, raw, 6, 10);
        HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_ST2, 1, &st1, 1, 10);

        int16_t mx = (int16_t)((raw[1] << 8) | raw[0]);
        int16_t my = (int16_t)((raw[3] << 8) | raw[2]);

        float mx_corr = (float)mx - mag_bias_x;
        float my_corr = (float)my - mag_bias_y;

        // AK8963 axes bi xoay 90° so voi gyro trong MPU9250:
        //   body_X = -sensor_Y  (right)
        //   body_Y =  sensor_X  (forward)
        //   body_Z =  sensor_Z  (up)
        // mag_yaw = atan2(body_X, body_Y)
        float mag_yaw = atan2f(-my_corr, mx_corr);

        // Complementary filter
        fused_yaw = CF_ALPHA * (fused_yaw + gyro_z_rads * TS_SENSOR)
                  + (1.0f - CF_ALPHA) * mag_yaw;
        fused_yaw = atan2f(sinf(fused_yaw), cosf(fused_yaw));
      } else {
        // Mag not ready: gyro-only prediction
        fused_yaw += gyro_z_rads * TS_SENSOR;
        fused_yaw = atan2f(sinf(fused_yaw), cosf(fused_yaw));
      }
    } else if (mpu_ok) {
      // No mag: gyro-only integration
      fused_yaw += gyro_z_rads * TS_SENSOR;
      fused_yaw = atan2f(sinf(fused_yaw), cosf(fused_yaw));
    }

    // Send DATA packet (format: DATA,rpmL,rpmR,gyro_z,fused_yaw)
    int len = snprintf(uart_buf, sizeof(uart_buf),
        "DATA,%d,%d,%d,%d\r\n",
        (int)(rpmL_signed * 10.0f),
        (int)(rpmR_signed * 10.0f),
        (int)(gyro_z_rads * 1000.0f),
        (int)(fused_yaw * 5729.57795f));  // rad * (180/pi * 100) = deg*100

    if (huart2.gState == HAL_UART_STATE_READY)
      HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_buf, (uint16_t)len);
  }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART2) return;

  if (Size < sizeof(rx_buffer)) rx_buffer[Size] = '\0';
  else rx_buffer[sizeof(rx_buffer)-1] = '\0';

  char *cmd = (char*)rx_buffer;

  /* ── CMD,v,w ── */
  if (strncmp(cmd, "CMD,", 4) == 0)
  {
    char *tok = strtok(cmd + 4, ",");
    float v = 0.0f, w = 0.0f;
    if (tok) { v = atof(tok); tok = strtok(NULL, ","); }
    if (tok)   w = atof(tok);

    float v_L = v - w * (WHEEL_BASE / 2.0f);
    float v_R = v + w * (WHEEL_BASE / 2.0f);

    float max_v = MAX_RPM * (PI * WHEEL_DIAMETER / 60.0f);
    if (v_L >  max_v) v_L =  max_v;
    if (v_L < -max_v) v_L = -max_v;
    if (v_R >  max_v) v_R =  max_v;
    if (v_R < -max_v) v_R = -max_v;

    if (v_L >= 0.0f) { dirL =  1; HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); }
    else             { dirL = -1; HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   v_L = -v_L; }

    if (v_R >= 0.0f) { dirR =  1; HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); }
    else             { dirR = -1; HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   v_R = -v_R; }

    pidL.setpoint = v_L * (60.0f / (PI * WHEEL_DIAMETER));
    pidR.setpoint = v_R * (60.0f / (PI * WHEEL_DIAMETER));
  }


  else if (strncmp(cmd, "STOP", 4) == 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_2,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    pidL.setpoint = 0; pidL.integral = 0; pidL.last_error = 0; pidL.output = 0;
    pidR.setpoint = 0; pidR.integral = 0; pidR.last_error = 0; pidR.output = 0;
  }

  memset(rx_buffer, 0, sizeof(rx_buffer));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  param: assert_param error line source number
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
