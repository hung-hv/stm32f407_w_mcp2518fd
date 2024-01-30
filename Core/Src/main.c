/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_spi.h"
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
//uint32_t counter_timer = 0;
uint8_t RX_Buffer[BUFFER_SIZE] = {0};
uint8_t *ptr_rx;
uint8_t RX_data = 0;
uint8_t TX_Buffer[BUFFER_SIZE] = {1,2,3,4,5,6,7,8,9,10};
uint32_t RX_index = 0;
uint8_t flag_send_frame = 0;
uint8_t flag_receive_frame = 0;
//uint8_t a=969696;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void blink_led(uint8_t times);

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
  MX_TIM14_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim14);
//  HAL_SPI_Receive_IT(&hspi1, RX_Buffer, BUFFER_SIZE);
//  HAL_SPI_Receive_IT(&hspi3, RX_Buffer, BUFFER_SIZE);
//  HAL_SPI_Receive_IT(&hspi3, RX_Buffer, BUFFER_SIZE);


//  HAL_SPI_Receive_IT(&hspi3, ptr_rx, 1);
//  HAL_SPI_Receive_IT(&hspi3, &RX_data, sizeof(RX_data));
//  HAL_SPI_Transmit(&hspi2, (uint8_t*)TX_Buffer, BUFFER_SIZE, 1000);
  HAL_SPI_Receive_IT(&hspi3, (uint8_t*)RX_Buffer, sizeof(RX_Buffer));


  CANFDSPI_Init();
//  DRV_SPI_Initialize();
//  DRV_SPI_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (flag_send_frame == 1) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_HIGH);
		  mcp2518fd_transmit();
		  //debounce
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_LOW);
		  flag_send_frame = 0;
	  }
	  if (flag_receive_frame == 1) {
		  mcp2518fd_receive();
		  blink_led(2);
		  flag_receive_frame = 0;
	  }
	  //	  if (flag_send_frame == 1) {
	  ////		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  ////		  mcp2518fd_transpond();
	  //		  RX_index=0;
	  //		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_HIGH);
	  ////		  mcp2518fd_transmit();
	  ////		  mcp2518fd_transpond();
	  ////		  mcp2518fd_receive();
	  ////		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  //		  mcp2518fd_transmit();
	  ////		  HAL_Delay(100);
	  ////		  CANFDSPI_Test();
	  ////		  mcp2518fd_receive();
	  ////		  RX_index=0;
	  ////		  for (uint16_t i = 0; i < BUFFER_SIZE; i++)
	  ////		      {
	  ////			  	  TX_Buffer[i] = rand() & 0xf;
	  ////		      }
	  ////		  HAL_SPI_Transmit(&hspi2, (uint8_t*)TX_Buffer, sizeof(TX_Buffer), 1000);
	  ////		  HAL_Delay(200);
	  ////		  HAL_SPI_Receive_IT(&hspi3, RX_Buffer, sizeof(RX_Buffer));
	  ////		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  ////		  HAL_SPI_Receive_IT(&hspi3, ptr_rx, 1);
	  ////		  HAL_SPI_Receive_IT(&hspi3, &RX_data, sizeof(RX_data));
	  ////		  HAL_SPI_Receive_IT(&hspi3, (uint8_t*)RX_Buffer, BUFFER_SIZE);
	  //		  HAL_Delay(100);
	  //
	  //		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_LOW);
	  //		  flag_send_frame = 0;
	  //	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void blink_led(uint8_t times) {
	for (int i = 0; i <= times; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_HIGH);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_LOW);
		HAL_Delay(50);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

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
