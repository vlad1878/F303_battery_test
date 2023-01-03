/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "GMG12864_lib.h"
#include "INA219_lib.h"
#include "fatfs_sd.h"
#include "string.h"
#include "max_ds3231_lib.h"
#include "SMA_filter_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define high_charge_on() (WRITE_REG(GPIOC->BSRR, GPIO_BSRR_BS_0))
#define low_charge_on() (WRITE_REG(GPIOC->BSRR, GPIO_BSRR_BS_1))
#define discharge_on() (WRITE_REG(GPIOC->BSRR, GPIO_BSRR_BS_2))
#define high_charge_off() (WRITE_REG(GPIOC->BSRR, GPIO_BSRR_BR_0))
#define low_charge_off() (WRITE_REG(GPIOC->BSRR, GPIO_BSRR_BR_1))
#define discharge_off() (WRITE_REG(GPIOC->BSRR, GPIO_BSRR_BR_2))
#define read_state_of_high_charge() (READ_BIT(GPIOC->IDR, GPIO_IDR_0))
#define read_state_of_low_charge() (READ_BIT(GPIOC->IDR, GPIO_IDR_1))
#define read_state_of_discharge() (READ_BIT(GPIOC->IDR,GPIO_IDR_2))
#define ADC_MAX 0xFFF
#define BATTERY_LOW_LIMIT 5100U
#define BATTERY_MEDIUM_LIMIT 7100U
#define BATTERY_HIGH_LIMIT 7300U
#define reaction 10
#define scroll 50
#define DELAY_LOW_CHARGE_IN_SEC 160U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int8_t A = 0;
volatile uint8_t status, status_old = 0x00;
volatile unsigned long Time, Time_old = 0;

volatile bool ERROR_GMG12864 = 0;
volatile bool ERROR_SD_CARD = 0;
volatile bool ERROR_INA219 = 0;
volatile bool ERROR_DS3231 = 0;

FATFS fs;
FIL fil;
FRESULT fresult;
char buffer_sd_card[1024];
uint16_t br, bw;
uint16_t counter_sd_card = 0;
uint16_t sd_card_execution_time = 0;
uint16_t t_sd_card_ex_time_start = 0;
uint16_t t_sd_card_ex_time_stop = 0;

INA219_t ina219;
extern char tx_buffer[128];
volatile bool ina219_flag = 0;
uint16_t v_bus, v_shunt, current, power;
volatile uint8_t display_mode = 0;
unsigned long t_ina219 = 0;
unsigned long t_gmg12864 = 0;
unsigned long t_sd_card = 0;
unsigned long t_button_mode_led = 0;
bool state_high_charge = 0;
bool state_low_charge = 0;
bool state_discharge = 0;
bool discharge_enable = 0;
volatile bool control_mode = 1;
uint8_t manual_mode = 0;
volatile bool flag_change_mode = 0;

extern uint8_t Seconds;
extern uint8_t Minutes;
extern uint8_t Hours;
extern uint8_t Day;
extern uint8_t Date;
extern uint8_t Month;
extern uint8_t Cuntury;
extern uint16_t Year;
extern float max_ds3231_temp;
unsigned long t_ds3231 = 0;

unsigned long t_init_gmg12864 = 0;
float Sensitivity = 0.066f;
float RawVoltage = 0.0f;
float Current_ASC712 = 0.0f;
extern bool adc_flag;
extern uint16_t ADC_SMA_Data_1;
extern uint16_t ADC_SMA_Data_2;
float filtered_voltage = 0.0f;

enum  Mode{HIGH_CHARGE, LOW_CHARGE, DISCHARGE};
uint8_t Current_Mode = 0;
uint8_t Prev_Mode = 2;

volatile uint16_t tim7_counter = 0;

float capacity = 0.0f;
bool capacity_flag = 0;
bool capacity_timer_flag_1 = 1;
uint32_t t_capacity = 0;
uint32_t time_start = 0;
uint32_t time_stop = 0;
<<<<<<< HEAD
uint32_t time_of_discharge_in_min = 0;
uint32_t discharge_time_min = 0;
uint32_t discharge_time_max = 0;
=======
<<<<<<< HEAD
uint32_t time_of_discharge_in_min = 0;
uint32_t discharge_time_min = 0;
uint32_t discharge_time_max = 0;
=======
float time_discharge = 0;
>>>>>>> 8c8e7ad93b10d43edf1f1f51e2b02297352c7e2b
>>>>>>> 663c995b2cc231ad7a531d14d4fcd9a60ffcf578

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void automatik_mode(void);
void get_param_from_ina219(void);
void mode_change_func(void);
void manual_mode_func(void);
void read_state_of_relays(void);
void print_gmg12864_level_1(void);
void GMG12864_first_line_level_1(uint8_t x, uint8_t y);
void GMG12864_second_line_level_1(uint8_t x, uint8_t y);
void GMG12864_third_line_level_1(uint8_t x, uint8_t y);
void GMG12864_fourth_line_level_1(uint8_t x, uint8_t y);
void GMG12864_fifth_line_level_1(uint8_t x, uint8_t y);
void GMG12864_sixth_line_level_1(uint8_t x, uint8_t y);
void GMG12864_first_line_level_2(uint8_t x, uint8_t y);
void GMG12864_second_line_level_2(uint8_t x, uint8_t y);
void GMG12864_third_line_level_2(uint8_t x, uint8_t y);
void GMG12864_fourth_line_level_2(uint8_t x, uint8_t y);
void GMG12864_fifth_line_level_2(uint8_t x, uint8_t y);
void GMG12864_sixth_line_level_2(uint8_t x, uint8_t y);
void GMG12864_first_line_level_3(uint8_t x, uint8_t y);
void GMG12864_second_line_level_3(uint8_t x, uint8_t y);
void GMG12864_third_line_level_3(uint8_t x, uint8_t y);
void GMG12864_fourth_line_level_3(uint8_t x, uint8_t y);
void GMG12864_fifth_line_level_3(uint8_t x, uint8_t y);
void GMG12864_sixth_line_level_3(uint8_t x, uint8_t y);
void sd_card_write(void);
void ds3231_get_time_and_temp(void);
float get_current_ASC712(void);
void button_mode_func(void);
void button_mode_led_delay(void);
void tim7_start(void);
void tim7_stop(void);
void get_capacity_of_battery(void);
void start_capacity_timer(void);
void stop_capacity_timer(void);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  GMG12864_Init();
  GMG12864_logo_demonstration();
  HAL_Delay(2000);
  INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
  t_ina219 = HAL_GetTick();
  t_gmg12864 = HAL_GetTick();
  t_sd_card = HAL_GetTick();
  t_ds3231 = HAL_GetTick();
  t_init_gmg12864 = HAL_GetTick();
  t_button_mode_led = HAL_GetTick();
  t_capacity = HAL_GetTick();
 // max_ds3231_set_hours(7);
  //max_ds3231_set_minutes(38);
  //max_ds3231_set_seconds(0);
  //max_ds3231_set_day(1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  HAL_Delay(500);
  if((fresult = f_mount(&fs, "", 0)) != FR_OK){
	  sprintf(buffer_sd_card, "Card is not detected!!");
	  WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS_5);
	  GMG12864_third_line_level_1(0, 0);
	  HAL_Delay(2000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  read_state_of_relays();
	  get_param_from_ina219();
	  ds3231_get_time_and_temp();
	  automatik_mode();
	  manual_mode_func();
	  mode_change_func();
	  print_gmg12864_level_1();
	  sd_card_write();
	  button_mode_func();
	  Current_ASC712 = get_current_ASC712();
	  get_capacity_of_battery();
	  if(HAL_GetTick() - t_init_gmg12864 > 100000){
		  t_init_gmg12864 = HAL_GetTick();
		  GMG12864_Init();
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, high_charge_relay_Pin|low_charge_relay_Pin|discharge_relay_Pin|led_high_charge_Pin
                          |led_discharge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|RST_Pin|DC_Pin|user_led_Pin
                          |CS_SD_CARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : user_button_Pin */
  GPIO_InitStruct.Pin = user_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(user_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : high_charge_relay_Pin low_charge_relay_Pin discharge_relay_Pin led_high_charge_Pin
                           led_discharge_Pin */
  GPIO_InitStruct.Pin = high_charge_relay_Pin|low_charge_relay_Pin|discharge_relay_Pin|led_high_charge_Pin
                          |led_discharge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led_low_charge_Pin PC8 PC9 */
  GPIO_InitStruct.Pin = led_low_charge_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin RST_Pin DC_Pin user_led_Pin
                           CS_SD_CARD_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RST_Pin|DC_Pin|user_led_Pin
                          |CS_SD_CARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Mode_Pin */
  GPIO_InitStruct.Pin = Button_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_Mode_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void manual_mode_func(){
	if(!control_mode){
		TIM3->CCR1 = 1000;
		manual_mode = A;
		switch(manual_mode){
		case 0:
			high_charge_off();
			low_charge_off();
			discharge_off();
			break;
		case 1:
			high_charge_on();
			low_charge_off();
			discharge_off();
			break;
		case 2:
			high_charge_off();
			low_charge_on();
			discharge_off();
			break;
		case 3:
			high_charge_off();
			low_charge_off();
			discharge_on();
			break;
		}
	}
}

void automatik_mode(){
	if(control_mode){
		button_mode_led_delay();
		if(v_bus < BATTERY_LOW_LIMIT && (discharge_enable == 0) && (Prev_Mode == DISCHARGE)){
			low_charge_off();
			discharge_off();
			high_charge_on();
			Prev_Mode = HIGH_CHARGE;
		}
		else if((v_bus > BATTERY_MEDIUM_LIMIT) && (discharge_enable == 0) && (Prev_Mode == HIGH_CHARGE)){
			high_charge_off();
			discharge_off();
			low_charge_on();
			tim7_start();
			Prev_Mode = LOW_CHARGE;
			discharge_enable = 1;
			//if(v_bus >= BATTERY_HIGH_LIMIT){
				//discharge_enable = 1;
				//Prev_Mode = LOW_CHARGE;
			//}
		}
		if(discharge_enable && (Prev_Mode == LOW_CHARGE) && (tim7_counter >= DELAY_LOW_CHARGE_IN_SEC)){
			tim7_stop();
			low_charge_off();
			high_charge_off();
			discharge_on();
			start_capacity_timer();
			if(v_bus < BATTERY_LOW_LIMIT){
				stop_capacity_timer();
				discharge_enable = 0;
				Prev_Mode = DISCHARGE;
				tim7_counter = 0;
				capacity_flag = 1;
				capacity_timer_flag_1 = 1;
			}
		}
	}
}

void get_param_from_ina219(){
	if(ina219_flag){
		ina219_flag = 0;
		filtered_voltage = (float)ADC_SMA_Data_2;
		v_bus = INA219_ReadBusVoltage(&ina219);
		v_shunt = INA219_ReadShuntVoltage(&ina219);
		current = INA219_ReadCurrent(&ina219);
		power = INA219_Read_Power(&ina219);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == (0x2000)){
		//flag_change_mode = 1;
		if((display_mode < 3) && (display_mode >= 0)){
			display_mode += 1;
		}
		else{
			display_mode = 0;
		}
	}
	if(GPIO_Pin == GPIO_PIN_8){
		if (!(GPIOC->IDR & GPIO_PIN_8) && (!(GPIOC->IDR & GPIO_PIN_9))) {
			status = 0x00;
		} else if ((GPIOC->IDR & GPIO_PIN_8) && (!(GPIOC->IDR & GPIO_PIN_9))) {
			status = 0x10;
		} else if ((GPIOC->IDR & GPIO_PIN_8) && (GPIOC->IDR & GPIO_PIN_9)) {
			status = 0x11;
		} else if (!(GPIOC->IDR & GPIO_PIN_8) && (GPIOC->IDR & GPIO_PIN_9)) {
			status = 0x01;
		}

		if (status_old == 0x10 && status == 0x11) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A + scroll;
				if(A > 3){
					A = 3;
				}
			} else {
				A = A + 1;
				if(A > 3){
					A = 3;
				}
			}
			Time_old = Time;
		} else if (status_old == 0x01 && status == 0x00) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A + scroll;
				if(A > 3){
					A = 3;
				}
			} else {
				A = A + 1;
				if(A > 3){
					A = 3;
				}
			}
			Time_old = Time;
		}

		if (status_old == 0x11 && status == 0x10) {
			status_old = 0x10;

		} else if (status_old == 0x00 && status == 0x01) {
			status_old = 0x01;

		}

		else if (status_old == 0x10 && status == 0x00) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A - scroll;
				if(A < 0){
					A = 0;
				}
			} else {
				A = A - 1;
				if(A < 0){
					A = 0;
				}
			}
			Time_old = Time;
		}

		else if (status_old == 0x01 && status == 0x11) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A - scroll;
				if(A < 0){
					A = 0;
				}
			} else {
				A = A - 1;
				if(A < 0){
					A = 0;
				}
			}
			Time_old = Time;
		}
		status_old = status;
	}
	if(GPIO_Pin == GPIO_PIN_9){
		if (!(GPIOC->IDR & GPIO_PIN_8) && (!(GPIOC->IDR & GPIO_PIN_9))) {
			status = 0x00;
		} else if ((GPIOC->IDR & GPIO_PIN_8) && (!(GPIOC->IDR & GPIO_PIN_9))) {
			status = 0x10;
		} else if ((GPIOC->IDR & GPIO_PIN_8) && (GPIOC->IDR & GPIO_PIN_9)) {
			status = 0x11;
		} else if (!(GPIOC->IDR & GPIO_PIN_8) && (GPIOC->IDR & GPIO_PIN_9)) {
			status = 0x01;
		}

		if (status_old == 0x10 && status == 0x11) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A + scroll;
				if(A > 3){
					A = 3;
				}
			} else {
				A = A + 1;
				if(A > 3){
					A = 3;
				}
			}
			Time_old = Time;
		} else if (status_old == 0x01 && status == 0x00) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A + scroll;
				if(A > 3){
					A = 3;
				}
			} else {
				A = A + 1;
				if(A > 3){
					A = 3;
				}
			}
			Time_old = Time;
		}

		if (status_old == 0x11 && status == 0x10) {
			status_old = 0x10;

		} else if (status_old == 0x00 && status == 0x01) {
			status_old = 0x01;

		}

		else if (status_old == 0x10 && status == 0x00) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A - scroll;
				if(A < 0){
					A = 0;
				}
			} else {
				A = A - 1;
				if(A < 0){
					A = 0;
				}
			}
			Time_old = Time;
		}

		else if (status_old == 0x01 && status == 0x11) {
			Time = HAL_GetTick();
			if (Time - Time_old < reaction) {
				A = A - scroll;
				if(A < 0){
					A = 0;
				}
			} else {
				A = A - 1;
				if(A < 0){
					A = 0;
				}
			}
			Time_old = Time;
		}
		status_old = status;
	}

}


void mode_change_func(){
	if(flag_change_mode){
		GMG12864_Clean_Frame_buffer();
		flag_change_mode = 0;
		high_charge_off();
		low_charge_off();
		discharge_off();
		A = 0;
		Prev_Mode = DISCHARGE;
	}
}

void button_mode_func(){
	if(!(READ_BIT(GPIOC->IDR, GPIO_IDR_7))){
		control_mode = 0;
	}
	else{
		control_mode = 1;
	}
}

void button_mode_led_delay(){
	if(control_mode){
		static uint16_t i = 0;
		static bool flag = 0;
		if((i <= 999) && (!flag) && (HAL_GetTick() - t_button_mode_led > 10)){
			t_button_mode_led = HAL_GetTick();
			i = 1000;
			TIM3->CCR1 = i;
			if(i == 1000){
				flag = 1;
			}
		}
		else if((flag == 1) && (HAL_GetTick() - t_button_mode_led > 10)){
			t_button_mode_led = HAL_GetTick();
			i = 0;
			TIM3->CCR1 = i;
			if(i <= 1){
				flag = 0;
			}
		}

	}
}

void read_state_of_relays(){
	state_high_charge = read_state_of_high_charge();
	state_low_charge = read_state_of_low_charge();
	state_discharge = read_state_of_discharge();
	if(state_high_charge){
		Current_Mode = HIGH_CHARGE;
	}
	else if(state_low_charge){
		Current_Mode = LOW_CHARGE;
	}
	else if(state_discharge){
		Current_Mode = DISCHARGE;
	}
}

void print_gmg12864_level_1(){
	if(((HAL_GetTick() - t_gmg12864) > 300) && (display_mode == 0)){
		t_gmg12864 = HAL_GetTick();
		GMG12864_first_line_level_1(0, 0);
		GMG12864_second_line_level_1(0, 10);
		GMG12864_third_line_level_1(0, 20);
		GMG12864_fourth_line_level_1(0, 30);
		GMG12864_fifth_line_level_1(0, 40);
		GMG12864_sixth_line_level_1(0, 50);
	}
	else if(((HAL_GetTick() - t_gmg12864) > 300) && (display_mode == 1)){
		t_gmg12864 = HAL_GetTick();
		GMG12864_first_line_level_2(0, 0);
		GMG12864_second_line_level_2(0, 10);
		GMG12864_third_line_level_2(0, 20);
		GMG12864_fourth_line_level_2(0, 30);
		GMG12864_fifth_line_level_2(0, 40);
		GMG12864_sixth_line_level_2(0, 50);
	}
	else if(((HAL_GetTick() - t_gmg12864) > 300) && (display_mode == 2)){
		t_gmg12864 = HAL_GetTick();
		GMG12864_first_line_level_3(0, 0);
		GMG12864_second_line_level_3(0, 10);
		GMG12864_third_line_level_3(0, 20);
		GMG12864_fourth_line_level_3(0, 30);
		GMG12864_fifth_line_level_3(0, 40);
		GMG12864_sixth_line_level_3(0, 50);
	}
}

void GMG12864_first_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Voltage is %d mV         ", v_bus);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_second_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Current is %.1f A    ", Current_ASC712);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_third_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "discharge %d         ", state_discharge);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_fourth_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "low charge %d         ", state_low_charge);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_fifth_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "high charge %d         ", state_high_charge);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_sixth_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Time is %d :%d :%d          ", Hours, Minutes, Seconds);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_first_line_level_2(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "SD CARD Parameters     ");
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_second_line_level_2(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Counter SD %d          ", counter_sd_card);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_third_line_level_2(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Timer low charge %d", tim7_counter);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_fourth_line_level_2(uint8_t x, uint8_t y){
<<<<<<< HEAD
	sprintf(tx_buffer, "Min. time = %d          ", discharge_time_min);
=======
<<<<<<< HEAD
	sprintf(tx_buffer, "Min. time = %d          ", discharge_time_min);
=======
	sprintf(tx_buffer, "Time discha. %.1f min.                     ", time_discharge);
>>>>>>> 8c8e7ad93b10d43edf1f1f51e2b02297352c7e2b
>>>>>>> 663c995b2cc231ad7a531d14d4fcd9a60ffcf578
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_fifth_line_level_2(uint8_t x, uint8_t y){
<<<<<<< HEAD
	sprintf(tx_buffer, "Max. time = &d           ", discharge_time_max);
=======
<<<<<<< HEAD
	sprintf(tx_buffer, "Max. time = &d           ", discharge_time_max);
=======
	sprintf(tx_buffer, "Capacity %.1f                      ", capacity);
>>>>>>> 8c8e7ad93b10d43edf1f1f51e2b02297352c7e2b
>>>>>>> 663c995b2cc231ad7a531d14d4fcd9a60ffcf578
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_sixth_line_level_2(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Time is %d : %d : %d                ", Hours, Minutes, Seconds);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_first_line_level_3(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "DS3231 Parameters            ");
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_second_line_level_3(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Time is %d : %d : %d                ", Hours, Minutes, Seconds);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_third_line_level_3(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Temp. DS3231 %.1f                ", max_ds3231_temp);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_fourth_line_level_3(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Day %d                ", Day);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_fifth_line_level_3(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Date %d              ", Date);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_sixth_line_level_3(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Month %d             ", Month);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void sd_card_write(){
	if(HAL_GetTick() - t_sd_card > 1000){
		t_sd_card_ex_time_start = HAL_GetTick();
		if((fresult = f_open(&fil, "parameters.txt", FA_OPEN_ALWAYS | FA_WRITE)) == FR_OK){
			t_sd_card = HAL_GetTick();
			sprintf(buffer_sd_card, "SD CARD OK!         ");
			fresult = f_lseek(&fil, fil.fsize);
			fresult = f_puts("DATA!!!/n", &fil);
			fresult = f_close(&fil);
			counter_sd_card += 1;
			SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_5);
		}
		else if((fresult = f_open(&fil, "parameters.txt", FA_OPEN_ALWAYS | FA_WRITE)) == FR_DISK_ERR){
			t_sd_card = HAL_GetTick();
			sprintf(buffer_sd_card, "SD CARD not detected!     ");
			SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5);
		}
		else{
			t_sd_card = HAL_GetTick();
			sprintf(buffer_sd_card, "SD CARD not communicate!!!    ");
			SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5);
		}
		t_sd_card_ex_time_stop = HAL_GetTick();
		sd_card_execution_time = t_sd_card_ex_time_stop - t_sd_card_ex_time_start;
	}
}

void ds3231_get_time_and_temp(){
	if(HAL_GetTick() - t_ds3231 > 1000){
		t_ds3231 = HAL_GetTick();
		max_ds3231_get_time();
		max_ds3231_get_temperature();
	}
}

float get_current_ASC712(){
	if(adc_flag){
		if((state_high_charge == 0) && (state_low_charge == 0) && (state_discharge == 0)){
			adc_flag = 0;
			float Current = 0.0f;
			RawVoltage = (float)ADC_SMA_Data_1 * 3.4f * 2.0f / (float)ADC_MAX;
			return Current = (RawVoltage - 2.43f) / Sensitivity;
		}
		else{
			adc_flag = 0;
			float Current = 0.0f;
			RawVoltage = (float)ADC_SMA_Data_1 * 3.4f * 2.0f / (float)ADC_MAX;
			return Current = (RawVoltage - 2.37f) / Sensitivity;
		}
	}
	else{
		return 0;
	}
}

void tim7_start(){
	HAL_TIM_Base_Start_IT(&htim7);
}

void tim7_stop(){
	HAL_TIM_Base_Stop_IT(&htim7);
	TIM7->CNT = 0;
}

void get_capacity_of_battery(){
	if(capacity_flag){
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 663c995b2cc231ad7a531d14d4fcd9a60ffcf578
		time_of_discharge_in_min = ((time_stop - time_start) / 1000) / 60;
		if(time_of_discharge_in_min > discharge_time_max){
			discharge_time_max = time_of_discharge_in_min;
		}
		else if(time_of_discharge_in_min < discharge_time_min){
			discharge_time_min = time_of_discharge_in_min;
		}
<<<<<<< HEAD
=======
=======
		time_discharge = ((time_stop - time_start) / (float)1000) / 60;
		capacity = 120 * ((float)time_discharge * (float)0.000000277) / (6 * (float)0.7);
>>>>>>> 8c8e7ad93b10d43edf1f1f51e2b02297352c7e2b
>>>>>>> 663c995b2cc231ad7a531d14d4fcd9a60ffcf578
		capacity_flag = 0;
	}
}
void start_capacity_timer(){
	if(capacity_timer_flag_1){
		time_start = HAL_GetTick();
		capacity_timer_flag_1 = 0;
	}
}

void stop_capacity_timer(){
	time_stop = HAL_GetTick();
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
