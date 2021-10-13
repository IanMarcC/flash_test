/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include <time.h>
#include  <errno.h>
#include  <sys/unistd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct MissionData
{
	uint8_t hours, minutes, seconds; //3 halfwords
	uint8_t * time;
	uint8_t gps_h, gps_m, gps_s; // 3 halfwords
	uint8_t * gps_time;

	uint8_t telemetry_on, sp1_released, sp2_released; //data for reset
	uint8_t sp1_tele, sp2_tele; //data for telemetry

	uint8_t sim_enabled, sim_active; //data for reset
	uint16_t sim_data; //Pressure, Resolution of 1 Pascal
	uint8_t flight_or_sim; // data for telemetry
	uint8_t packet_num;
	uint32_t altitude; //doubleword
	uint32_t gps_altitude;
	uint32_t temperature;
	uint32_t voltage;
	uint32_t gps_lat, gps_long;
	uint8_t gps_sats;
	uint8_t apogee;
	uint8_t * state;
	uint8_t * last_cmd;
};

typedef struct TestStruct
{
	char test_string[10];
	char * test_string_pointer;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEAM_ID 2307 //2302 + 5, per requirements
#define TEMP_AT_SEA_LEVEL 20
#define TEMP_LAPSE_RATE 0.00065
#define PRESS_AT_SEA_LEVEL 99000

#define RESET_FLAG ((uint32_t) 0x0F0F0F0F)
#define ADDR_PAGE_126 ((uint32_t) 0x0801F800)
//#define ADDR_PAGE_LAST ((uint32_t) 0x800)
#define MD_SIZE (int)20
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile struct MissionData mission_data = {
		11,
		22,
		33,
		0,
		21,
		32,
		43,
		0,
		1, //telemetry_on
		0, //sp1_released
		0, //sp2_released
		0,
		0,
		1, //sim_enabled
		1, //sim_active
		101325, //sim_data
		1, //sim
		156,
		97000,
		96970.2, //gps_altitude
		32.3, //temp in celcius
		4.79, //voltage
		101.03,
		222.42,
		3, //gps_sats
		0,
		0,
		0
};

char uart_buf[128];
uint8_t pin_state = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
	{
		ITM_SendChar(*ptr++);
	}
   return len;
}

//
//
//This function overwrites the Flash memory
//STM32F103 is a medium-density device
//128 pages of 1Kbyte memory
//Starting at page 126 (addr 0x0801_F800)
void save_mission_data(struct MissionData * md) //Involves Flash Pages, using HAL_Flash, not EEPROM emulation
{
	FLASH_EraseInitTypeDef erase_struct;
	uint8_t buf[64];
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    |
	                       FLASH_FLAG_WRPERR |
	                       FLASH_FLAG_PGERR);
	uint32_t page_error = 0;
	//char * message = "inside save mission data\r\n";
	//HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
	erase_struct.PageAddress = ADDR_PAGE_126;
	erase_struct.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_struct.NbPages = 1;
	//Unlock first
	HAL_FLASH_Unlock();

	//Erase page
	if (HAL_FLASHEx_Erase(&erase_struct, &page_error) != HAL_OK)
	{
		strcpy((char*)buf, "Flash Erase Error \r\n");
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
	}
	//HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
	//Write (aka program) page
	//half word 16-bit = 2 bytes, which is the minimum writable size for the STM32F1xx series
	uint8_t offset = 0;

	//
	//There's an elegant loop possible if I knew C's struct memory padding better
	//But I'm hardcoding it instead
	//
	//for (int i = 0, i < MD_SIZE, i++)
	//	{
	//	}
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126, RESET_FLAG);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->hours);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->minutes);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->seconds);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->telemetry_on);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->sp1_released);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->sp2_released);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->sim_enabled);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->sim_active);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->flight_or_sim);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->packet_num);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->altitude);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->temperature);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->voltage);
	offset += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,ADDR_PAGE_126+offset, md->apogee);

	HAL_FLASH_Lock();
	//message = "after FLASH lock";
	//HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
}
void check_reset(struct MissionData * md)
{
	uint32_t check = *(__IO uint32_t *)ADDR_PAGE_126;
	uint8_t message[32] = "Reset present\r\n";
	if(check == RESET_FLAG)
	{
		HAL_UART_Transmit(&huart1, (char *)message, strlen((char *)message), HAL_MAX_DELAY);
		//execute_reset(md);
	}
	else
	{
		strcpy((char *)message,"Reset error\r\n");
		HAL_UART_Transmit(&huart1, (char *)message, strlen((char *)message), HAL_MAX_DELAY);
	}
}

void execute_reset(struct MissionData * md)
{
	uint32_t offset = 4;
	md->hours = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->minutes = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->seconds = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->telemetry_on = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->sp1_released = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->sp2_released = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->sim_enabled = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->sim_active = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->flight_or_sim = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->packet_num = *(__IO uint8_t *)(ADDR_PAGE_126 + offset);
	offset += 2;
	md->altitude = *(__IO uint32_t *)(ADDR_PAGE_126 + offset);
	offset += 4;
	md->temperature = *(__IO uint32_t *)(ADDR_PAGE_126 + offset);
	offset += 4;
	md->voltage = *(__IO uint32_t *)(ADDR_PAGE_126 + offset);
	offset += 4;
	md->apogee = *(__IO uint32_t *)(ADDR_PAGE_126 + offset);
	//
	//Need to add derived data
	//
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t message[21] = "TIMER 2 VALUE: %d";

	uint16_t tim2_val = 0;

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  struct TestStruct test1 = {"Hello!\r\n","This can vary\r\n"};
  	    char * buf;
  	    char * template = "Fixed:%s\r\nVariable:%s\r\n";

  test1.test_string_pointer = "This can vary widely, so be careful";

  double tester = -111.546789;
  char * tester_str;
  while (1)
  {
	  //snprintf(tester_str, 11, "%3.4f\n", tester);
	 // snprintf(uart_buf, 128, "TIMER 2 VALUE: %d\r\n", __HAL_TIM_GET_COUNTER(&htim2));
	 // HAL_UART_Transmit(&huart1, tester_str, 11, HAL_MAX_DELAY);

	  //HAL_Delay(1000);
	 // snprintf(buf,strlen(buf),template, test1.test_string,test1.test_string_pointer);

	  HAL_UART_Transmit(&huart1, "In main\n", 10,HAL_MAX_DELAY);
	  HAL_Delay(1000);
		 check_reset(&mission_data);
		 save_mission_data(&mission_data);
		 //tim2_val = __HAL_TIM_GET_COUNTER(&htim2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  //__HAL_TIM_ENABLE_IT(&htim2,);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	static uint8_t packetnum = 0;
  UNUSED(htim);
  if(htim == &htim2)
  {

	  pin_state = !pin_state;
	 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, pin_state);
	 snprintf(uart_buf, 128, "INTERRUPT TIMER 2 MESSAGE NUM: %d\r\n", packetnum);
	 //HAL_UART_Transmit(&huart1, uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
	 packetnum++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
