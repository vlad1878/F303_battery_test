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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define high_charge_on() (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS_0))
#define low_charge_on() (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS_1))
#define discharge_on() (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS_2))
#define high_charge_off() (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR_0))
#define low_charge_off() (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR_1))
#define discharge_off() (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR_2))
#define read_state_of_high_charge() (READ_BIT(GPIOC->IDR, GPIO_IDR_0))
#define read_state_of_low_charge() (READ_BIT(GPIOC->IDR, GPIO_IDR_1))
#define read_state_of_discharge() (READ_BIT(GPIOC->IDR,GPIO_IDR_2))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FATFS fs;
FIL fil;
FRESULT fresult;
char buffer_sd_card[1024];
uint16_t br, bw;
uint16_t counter_sd_card = 0;
volatile bool sd_card_init_flag = 0;

INA219_t ina219;
extern char tx_buffer[128];
uint16_t v_bus, v_shunt, current, power;
unsigned long t_ina219 = 0;
unsigned long t_gmg12864 = 0;
unsigned long t_sd_card = 0;
bool state_high_charge = 0;
bool state_low_charge = 0;
bool state_discharge = 0;
bool discharge_enable = 0;
volatile bool control_mode = 0;
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
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
void sd_card_write(void);
void manual_init_sd_card(void);
void ds3231_get_time_and_temp(void);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  GMG12864_Init();
  INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
  t_ina219 = HAL_GetTick();
  t_gmg12864 = HAL_GetTick();
  t_sd_card = HAL_GetTick();
  t_ds3231 = HAL_GetTick();
  HAL_Delay(500);
  if((fresult = f_mount(&fs, "", 0)) != FR_OK){
	  sprintf(buffer_sd_card, "Card is not detected!!");
	  SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5);
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
	  manual_init_sd_card();
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
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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

  /*Configure GPIO pin : led_low_charge_Pin */
  GPIO_InitStruct.Pin = led_low_charge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(led_low_charge_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin RST_Pin DC_Pin user_led_Pin
                           CS_SD_CARD_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RST_Pin|DC_Pin|user_led_Pin
                          |CS_SD_CARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void manual_mode_func(){
	if(control_mode){
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
	if(!control_mode){
		if(v_bus < 800 && (discharge_enable == 0)){
			low_charge_off();
			discharge_off();
			high_charge_on();
		}
		else if(v_bus > 3100 && (discharge_enable == 0)){
			high_charge_off();
			discharge_off();
			low_charge_on();
			if(v_bus >= 4250){
				discharge_enable = 1;
			}
		}
		if(discharge_enable){
			low_charge_off();
			high_charge_off();
			discharge_on();
			if(v_bus <= 5100){
				discharge_enable = 0;
			}
		}
	}
}

void get_param_from_ina219(){
	if((HAL_GetTick() - t_ina219) > 100){
		t_ina219 = HAL_GetTick();
		v_bus = INA219_ReadBusVoltage(&ina219);
		v_shunt = INA219_ReadShuntVoltage(&ina219);
		current = INA219_ReadCurrent(&ina219);
		power = INA219_Read_Power(&ina219);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == (0x2000)){
		//control_mode = !control_mode;
		//flag_change_mode = 1;
		sd_card_init_flag = 1;
	}
}

void mode_change_func(){
	if(flag_change_mode){
		flag_change_mode = 0;
		high_charge_off();
		low_charge_off();
		discharge_off();
	}
}

void read_state_of_relays(){
	state_high_charge = read_state_of_high_charge();
	state_low_charge = read_state_of_low_charge();
	state_discharge = read_state_of_discharge();
}

void print_gmg12864_level_1(){
	if((HAL_GetTick() - t_gmg12864) > 200){
		t_gmg12864 = HAL_GetTick();
		GMG12864_first_line_level_1(0, 0);
		GMG12864_second_line_level_1(0, 10);
		GMG12864_third_line_level_1(0, 20);
		GMG12864_fourth_line_level_1(0, 30);
		GMG12864_fifth_line_level_1(0, 40);
		GMG12864_sixth_line_level_1(0, 50);
	}
}

void GMG12864_first_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Voltage is %d         ", v_bus);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_second_line_level_1(uint8_t x, uint8_t y){
	sprintf(tx_buffer, "Current is %d         ", current);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void GMG12864_third_line_level_1(uint8_t x, uint8_t y){
	//sprintf(tx_buffer, "Power is %d           ", power);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, buffer_sd_card);
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
	sprintf(tx_buffer, "discharge %d         ", state_discharge);
	GMG12864_Decode_UTF8(x, y, 1, inversion_off, tx_buffer);
	GMG12864_Update();
}

void sd_card_write(){
	if(HAL_GetTick() - t_sd_card > 1000){
		if((fresult = f_open(&fil, "parameters.txt", FA_OPEN_ALWAYS | FA_WRITE)) == FR_OK){
			t_sd_card = HAL_GetTick();
			sprintf(buffer_sd_card, "counter sd %d", counter_sd_card);
			fresult = f_lseek(&fil, fil.fsize);
			fresult = f_puts("DATA!!!/n", &fil);
			fresult = f_close(&fil);
			counter_sd_card += 1;
			SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_5);
		}
		else if((fresult = f_open(&fil, "parameters.txt", FA_OPEN_ALWAYS | FA_WRITE)) == FR_DISK_ERR){
			t_sd_card = HAL_GetTick();
			sprintf(buffer_sd_card, "Card is not detected!!");
			SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5);
		}
		else{
			t_sd_card = HAL_GetTick();
			sprintf(buffer_sd_card, "Problem with sd card!!");
			SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_5);
		}
	}
}

void manual_init_sd_card(){
	if(sd_card_init_flag){
		sd_card_init_flag = 0;
		memset(&fs, 0, sizeof(fs));
		fresult = f_mount(&fs, "", 0);
		HAL_Delay(50);
		fresult = f_mount(&fs, "", 0);
		HAL_Delay(50);
		fresult = f_mount(&fs, "", 0);
		HAL_Delay(50);
		fresult = f_mount(&fs, "", 0);
		HAL_Delay(50);
	}
}

void ds3231_get_time_and_temp(){
	if(HAL_GetTick() - t_ds3231 > 1000){
		t_ds3231 = HAL_GetTick();
		max_ds3231_get_time();
		max_ds3231_get_time();
	}
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
